#!/usr/bin/python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""This file generates shell code for the setup.SHELL scripts to set environment variables."""

from __future__ import print_function

import argparse
import copy
import errno
import os
import platform
import sys

CATKIN_MARKER_FILE = '.catkin'

system = platform.system()
IS_DARWIN = (system == 'Darwin')
IS_WINDOWS = (system == 'Windows')

PATH_TO_ADD_SUFFIX = ['bin']
if IS_WINDOWS:
    # while catkin recommends putting dll's into bin, 3rd party packages often put dll's into lib
    # since Windows finds dll's via the PATH variable, prepend it with path to lib
    PATH_TO_ADD_SUFFIX.extend(['lib'])

# subfolder of workspace prepended to CMAKE_PREFIX_PATH
ENV_VAR_SUBFOLDERS = {
    'CMAKE_PREFIX_PATH': '',
    'LD_LIBRARY_PATH' if not IS_DARWIN else 'DYLD_LIBRARY_PATH': 'lib',
    'PATH': PATH_TO_ADD_SUFFIX,
    'PKG_CONFIG_PATH': os.path.join('lib', 'pkgconfig'),
    'PYTHONPATH': 'lib/python2.7/dist-packages',
}


def rollback_env_variables(environ, env_var_subfolders):
    """
    Generate shell code to reset environment variables.

    by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH.
    This does not cover modifications performed by environment hooks.
    """
    lines = []
    unmodified_environ = copy.copy(environ)
    for key in sorted(env_var_subfolders.keys()):
        subfolders = env_var_subfolders[key]
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        value = _rollback_env_variable(unmodified_environ, key, subfolders)
        if value is not None:
            environ[key] = value
            lines.append(assignment(key, value))
    if lines:
        lines.insert(0, comment('reset environment variables by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH'))
    return lines


def _rollback_env_variable(environ, name, subfolders):
    """
    For each catkin workspace in CMAKE_PREFIX_PATH remove the first entry from env[NAME] matching workspace + subfolder.

    :param subfolders: list of str '' or subfoldername that may start with '/'
    :returns: the updated value of the environment variable.
    """
    value = environ[name] if name in environ else ''
    env_paths = [path for path in value.split(os.pathsep) if path]
    value_modified = False
    for subfolder in subfolders:
        if subfolder:
            if subfolder.startswith(os.path.sep) or (os.path.altsep and subfolder.startswith(os.path.altsep)):
                subfolder = subfolder[1:]
            if subfolder.endswith(os.path.sep) or (os.path.altsep and subfolder.endswith(os.path.altsep)):
                subfolder = subfolder[:-1]
        for ws_path in _get_workspaces(environ, include_fuerte=True, include_non_existing=True):
            path_to_find = os.path.join(ws_path, subfolder) if subfolder else ws_path
            path_to_remove = None
            for env_path in env_paths:
                env_path_clean = env_path[:-1] if env_path and env_path[-1] in [os.path.sep, os.path.altsep] else env_path
                if env_path_clean == path_to_find:
                    path_to_remove = env_path
                    break
            if path_to_remove:
                env_paths.remove(path_to_remove)
                value_modified = True
    new_value = os.pathsep.join(env_paths)
    return new_value if value_modified else None


def _get_workspaces(environ, include_fuerte=False, include_non_existing=False):
    """
    Based on CMAKE_PREFIX_PATH return all catkin workspaces.

    :param include_fuerte: The flag if paths starting with '/opt/ros/fuerte' should be considered workspaces, ``bool``
    """
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    value = environ[env_name] if env_name in environ else ''
    paths = [path for path in value.split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE)) or (include_fuerte and path.startswith('/opt/ros/fuerte')) or (include_non_existing and not os.path.exists(path))]
    return workspaces


def prepend_env_variables(environ, env_var_subfolders, workspaces):
    """Generate shell code to prepend environment variables for the all workspaces."""
    lines = []
    lines.append(comment('prepend folders of workspaces to environment variables'))

    paths = [path for path in workspaces.split(os.pathsep) if path]

    prefix = _prefix_env_variable(environ, 'CMAKE_PREFIX_PATH', paths, '')
    lines.append(prepend(environ, 'CMAKE_PREFIX_PATH', prefix))

    for key in sorted(key for key in env_var_subfolders.keys() if key != 'CMAKE_PREFIX_PATH'):
        subfolder = env_var_subfolders[key]
        prefix = _prefix_env_variable(environ, key, paths, subfolder)
        lines.append(prepend(environ, key, prefix))
    return lines


def _prefix_env_variable(environ, name, paths, subfolders):
    """
    Return the prefix to prepend to the environment variable NAME.

    Adding any path in NEW_PATHS_STR without creating duplicate or empty items.
    """
    value = environ[name] if name in environ else ''
    environ_paths = [path for path in value.split(os.pathsep) if path]
    checked_paths = []
    for path in paths:
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        for subfolder in subfolders:
            path_tmp = path
            if subfolder:
                path_tmp = os.path.join(path_tmp, subfolder)
            # skip nonexistent paths
            if not os.path.exists(path_tmp):
                continue
            # exclude any path already in env and any path we already added
            if path_tmp not in environ_paths and path_tmp not in checked_paths:
                checked_paths.append(path_tmp)
    prefix_str = os.pathsep.join(checked_paths)
    if prefix_str != '' and environ_paths:
        prefix_str += os.pathsep
    return prefix_str


def assignment(key, value):
    if not IS_WINDOWS:
        return 'export %s="%s"' % (key, value)
    else:
        return 'set %s=%s' % (key, value)


def comment(msg):
    if not IS_WINDOWS:
        return '# %s' % msg
    else:
        return 'REM %s' % msg


def prepend(environ, key, prefix):
    if key not in environ or not environ[key]:
        return assignment(key, prefix)
    if not IS_WINDOWS:
        return 'export %s="%s$%s"' % (key, prefix, key)
    else:
        return 'set %s=%s%%%s%%' % (key, prefix, key)


def find_env_hooks(environ, cmake_prefix_path):
    """Generate shell code with found environment hooks for the all workspaces."""
    lines = []
    lines.append(comment('found environment hooks in workspaces'))

    generic_env_hooks = []
    generic_env_hooks_workspace = []
    specific_env_hooks = []
    specific_env_hooks_workspace = []
    generic_env_hooks_by_filename = {}
    specific_env_hooks_by_filename = {}
    generic_env_hook_ext = 'bat' if IS_WINDOWS else 'sh'
    specific_env_hook_ext = environ['CATKIN_SHELL'] if not IS_WINDOWS and 'CATKIN_SHELL' in environ and environ['CATKIN_SHELL'] else None
    # remove non-workspace paths
    workspaces = [path for path in cmake_prefix_path.split(os.pathsep) if path and os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE))]
    for workspace in reversed(workspaces):
        env_hook_dir = os.path.join(workspace, 'etc', 'catkin', 'profile.d')
        if os.path.isdir(env_hook_dir):
            for filename in sorted(os.listdir(env_hook_dir)):
                if filename.endswith('.%s' % generic_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in generic_env_hooks_by_filename:
                        i = generic_env_hooks.index(generic_env_hooks_by_filename[filename])
                        generic_env_hooks.pop(i)
                        generic_env_hooks_workspace.pop(i)
                    # append env hook
                    generic_env_hooks.append(os.path.join(env_hook_dir, filename))
                    generic_env_hooks_workspace.append(workspace)
                    generic_env_hooks_by_filename[filename] = generic_env_hooks[-1]
                elif specific_env_hook_ext is not None and filename.endswith('.%s' % specific_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in specific_env_hooks_by_filename:
                        i = specific_env_hooks.index(specific_env_hooks_by_filename[filename])
                        specific_env_hooks.pop(i)
                        specific_env_hooks_workspace.pop(i)
                    # append env hook
                    specific_env_hooks.append(os.path.join(env_hook_dir, filename))
                    specific_env_hooks_workspace.append(workspace)
                    specific_env_hooks_by_filename[filename] = specific_env_hooks[-1]
    env_hooks = generic_env_hooks + specific_env_hooks
    env_hooks_workspace = generic_env_hooks_workspace + specific_env_hooks_workspace
    count = len(env_hooks)
    lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_COUNT', count))
    for i in range(count):
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d' % i, env_hooks[i]))
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d_WORKSPACE' % i, env_hooks_workspace[i]))
    return lines


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    parser.add_argument('--extend', action='store_true', help='Skip unsetting previous environment variables to extend context')
    parser.add_argument('--local', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    return parser.parse_known_args(args=args)[0]


if __name__ == '__main__':
    try:
        try:
            args = _parse_arguments()
        except Exception as e:
            print(e, file=sys.stderr)
            sys.exit(1)

        if not args.local:
            # environment at generation time
            CMAKE_PREFIX_PATH = r'/home/ubuntu/ros_catkin_ws/devel_isolated/visualization_marker_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/urdf_tutorial;/home/ubuntu/ros_catkin_ws/devel_isolated/urdf_robot;/home/ubuntu/ros_catkin_ws/devel_isolated/rviz_python_tutorial;/home/ubuntu/ros_catkin_ws/devel_isolated/rviz_plugin_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_rviz;/home/ubuntu/ros_catkin_ws/devel_isolated/librviz_tutorial;/home/ubuntu/ros_catkin_ws/devel_isolated/rviz;/home/ubuntu/ros_catkin_ws/devel_isolated/robot_state_publisher;/home/ubuntu/ros_catkin_ws/devel_isolated/kdl_parser;/home/ubuntu/ros_catkin_ws/devel_isolated/urdf;/home/ubuntu/ros_catkin_ws/devel_isolated/turtle_tf2;/home/ubuntu/ros_catkin_ws/devel_isolated/turtle_tf;/home/ubuntu/ros_catkin_ws/devel_isolated/turtle_actionlib;/home/ubuntu/ros_catkin_ws/devel_isolated/turtlesim;/home/ubuntu/ros_catkin_ws/devel_isolated/tf_conversions;/home/ubuntu/ros_catkin_ws/devel_isolated/tf2_kdl;/home/ubuntu/ros_catkin_ws/devel_isolated/laser_geometry;/home/ubuntu/ros_catkin_ws/devel_isolated/tf2_geometry_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/interactive_marker_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/interactive_markers;/home/ubuntu/ros_catkin_ws/devel_isolated/tf;/home/ubuntu/ros_catkin_ws/devel_isolated/tf2_ros;/home/ubuntu/ros_catkin_ws/devel_isolated/stereo_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_image_view;/home/ubuntu/ros_catkin_ws/devel_isolated/map_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/image_transport;/home/ubuntu/ros_catkin_ws/devel_isolated/cv_bridge;/home/ubuntu/ros_catkin_ws/devel_isolated/sensor_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_launch;/home/ubuntu/ros_catkin_ws/devel_isolated/roswtf;/home/ubuntu/ros_catkin_ws/devel_isolated/nodelet_topic_tools;/home/ubuntu/ros_catkin_ws/devel_isolated/diagnostic_analysis;/home/ubuntu/ros_catkin_ws/devel_isolated/actionlib_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/actionlib;/home/ubuntu/ros_catkin_ws/devel_isolated/rosbag;/home/ubuntu/ros_catkin_ws/devel_isolated/topic_tools;/home/ubuntu/ros_catkin_ws/devel_isolated/tf2_py;/home/ubuntu/ros_catkin_ws/devel_isolated/smach_ros;/home/ubuntu/ros_catkin_ws/devel_isolated/self_test;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_reconfigure;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_gui_py;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_gui_cpp;/home/ubuntu/ros_catkin_ws/devel_isolated/rostopic;/home/ubuntu/ros_catkin_ws/devel_isolated/rospy_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/rosnode;/home/ubuntu/ros_catkin_ws/devel_isolated/rosmsg;/home/ubuntu/ros_catkin_ws/devel_isolated/rosbag_storage;/home/ubuntu/ros_catkin_ws/devel_isolated/message_filters;/home/ubuntu/ros_catkin_ws/devel_isolated/kdl_parser_py;/home/ubuntu/ros_catkin_ws/devel_isolated/joint_state_publisher;/home/ubuntu/ros_catkin_ws/devel_isolated/filters;/home/ubuntu/ros_catkin_ws/devel_isolated/dynamic_reconfigure;/home/ubuntu/ros_catkin_ws/devel_isolated/diagnostic_common_diagnostics;/home/ubuntu/ros_catkin_ws/devel_isolated/diagnostic_updater;/home/ubuntu/ros_catkin_ws/devel_isolated/diagnostic_aggregator;/home/ubuntu/ros_catkin_ws/devel_isolated/rosout;/home/ubuntu/ros_catkin_ws/devel_isolated/roscpp_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/pluginlib_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/nodelet_tutorial_math;/home/ubuntu/ros_catkin_ws/devel_isolated/nodelet;/home/ubuntu/ros_catkin_ws/devel_isolated/bondpy;/home/ubuntu/ros_catkin_ws/devel_isolated/bondcpp;/home/ubuntu/ros_catkin_ws/devel_isolated/roscpp;/home/ubuntu/ros_catkin_ws/devel_isolated/xmlrpcpp;/home/ubuntu/ros_catkin_ws/devel_isolated/webkit_dependency;/home/ubuntu/ros_catkin_ws/devel_isolated/viz;/home/ubuntu/ros_catkin_ws/devel_isolated/visualization_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/visualization_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/urdfdom_py;/home/ubuntu/ros_catkin_ws/devel_isolated/urdf_parser_plugin;/home/ubuntu/ros_catkin_ws/devel_isolated/control_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/trajectory_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/tf2;/home/ubuntu/ros_catkin_ws/devel_isolated/tf2_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/std_srvs;/home/ubuntu/ros_catkin_ws/devel_isolated/smach_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/shape_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_py_common;/home/ubuntu/ros_catkin_ws/devel_isolated/rosgraph_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/nav_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/kdl_conversions;/home/ubuntu/ros_catkin_ws/devel_isolated/eigen_conversions;/home/ubuntu/ros_catkin_ws/devel_isolated/geometry_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/diagnostic_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/bond;/home/ubuntu/ros_catkin_ws/devel_isolated/actionlib_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/std_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/smclib;/home/ubuntu/ros_catkin_ws/devel_isolated/smach;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_web;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_topic;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_top;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_tf_tree;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_srv;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_shell;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_service_caller;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_runtime_monitor;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_robot_steering;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_robot_plugins;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_robot_monitor;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_robot_dashboard;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_py_console;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_publisher;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_pose_view;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_plot;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_nav_view;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_msg;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_moveit;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_logger_level;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_gui;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_graph;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_dep;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_console;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_common_plugins;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_bag_plugins;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_bag;/home/ubuntu/ros_catkin_ws/devel_isolated/rqt_action;/home/ubuntu/ros_catkin_ws/devel_isolated/rostest;/home/ubuntu/ros_catkin_ws/devel_isolated/roslz4;/home/ubuntu/ros_catkin_ws/devel_isolated/rosconsole_bridge;/home/ubuntu/ros_catkin_ws/devel_isolated/resource_retriever;/home/ubuntu/ros_catkin_ws/devel_isolated/qt_gui_cpp;/home/ubuntu/ros_catkin_ws/devel_isolated/pluginlib;/home/ubuntu/ros_catkin_ws/devel_isolated/rosconsole;/home/ubuntu/ros_catkin_ws/devel_isolated/angles;/home/ubuntu/ros_catkin_ws/devel_isolated/rosunit;/home/ubuntu/ros_catkin_ws/devel_isolated/roslaunch;/home/ubuntu/ros_catkin_ws/devel_isolated/python_qt_binding;/home/ubuntu/ros_catkin_ws/devel_isolated/roscpp_serialization;/home/ubuntu/ros_catkin_ws/devel_isolated/rostime;/home/ubuntu/ros_catkin_ws/devel_isolated/rosservice;/home/ubuntu/ros_catkin_ws/devel_isolated/rospy;/home/ubuntu/ros_catkin_ws/devel_isolated/rosparam;/home/ubuntu/ros_catkin_ws/devel_isolated/roslib;/home/ubuntu/ros_catkin_ws/devel_isolated/rospack;/home/ubuntu/ros_catkin_ws/devel_isolated/rosmaster;/home/ubuntu/ros_catkin_ws/devel_isolated/rosmake;/home/ubuntu/ros_catkin_ws/devel_isolated/roslisp;/home/ubuntu/ros_catkin_ws/devel_isolated/roslint;/home/ubuntu/ros_catkin_ws/devel_isolated/roslang;/home/ubuntu/ros_catkin_ws/devel_isolated/rosgraph;/home/ubuntu/ros_catkin_ws/devel_isolated/roscreate;/home/ubuntu/ros_catkin_ws/devel_isolated/roscpp_traits;/home/ubuntu/ros_catkin_ws/devel_isolated/roscpp_core;/home/ubuntu/ros_catkin_ws/devel_isolated/rosclean;/home/ubuntu/ros_catkin_ws/devel_isolated/rosbuild;/home/ubuntu/ros_catkin_ws/devel_isolated/rosboost_cfg;/home/ubuntu/ros_catkin_ws/devel_isolated/rosbash;/home/ubuntu/ros_catkin_ws/devel_isolated/rosbag_migration_rule;/home/ubuntu/ros_catkin_ws/devel_isolated/ros_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/ros_environment;/home/ubuntu/ros_catkin_ws/devel_isolated/ros_core;/home/ubuntu/ros_catkin_ws/devel_isolated/ros_comm;/home/ubuntu/ros_catkin_ws/devel_isolated/ros_base;/home/ubuntu/ros_catkin_ws/devel_isolated/ros;/home/ubuntu/ros_catkin_ws/devel_isolated/robot;/home/ubuntu/ros_catkin_ws/devel_isolated/qwt_dependency;/home/ubuntu/ros_catkin_ws/devel_isolated/qt_gui_py_common;/home/ubuntu/ros_catkin_ws/devel_isolated/qt_gui;/home/ubuntu/ros_catkin_ws/devel_isolated/qt_dotgraph;/home/ubuntu/ros_catkin_ws/devel_isolated/python_orocos_kdl;/home/ubuntu/ros_catkin_ws/devel_isolated/orocos_kdl;/home/ubuntu/ros_catkin_ws/devel_isolated/nodelet_core;/home/ubuntu/ros_catkin_ws/devel_isolated/mk;/home/ubuntu/ros_catkin_ws/devel_isolated/message_runtime;/home/ubuntu/ros_catkin_ws/devel_isolated/message_generation;/home/ubuntu/ros_catkin_ws/devel_isolated/media_export;/home/ubuntu/ros_catkin_ws/devel_isolated/gl_dependency;/home/ubuntu/ros_catkin_ws/devel_isolated/geometry_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/geometry;/home/ubuntu/ros_catkin_ws/devel_isolated/executive_smach;/home/ubuntu/ros_catkin_ws/devel_isolated/diagnostics;/home/ubuntu/ros_catkin_ws/devel_isolated/desktop;/home/ubuntu/ros_catkin_ws/devel_isolated/cpp_common;/home/ubuntu/ros_catkin_ws/devel_isolated/common_tutorials;/home/ubuntu/ros_catkin_ws/devel_isolated/common_msgs;/home/ubuntu/ros_catkin_ws/devel_isolated/class_loader;/home/ubuntu/ros_catkin_ws/devel_isolated/cmake_modules;/home/ubuntu/ros_catkin_ws/devel_isolated/bond_core;/home/ubuntu/ros_catkin_ws/devel_isolated/genpy;/home/ubuntu/ros_catkin_ws/devel_isolated/gennodejs;/home/ubuntu/ros_catkin_ws/devel_isolated/genlisp;/home/ubuntu/ros_catkin_ws/devel_isolated/geneus;/home/ubuntu/ros_catkin_ws/devel_isolated/gencpp;/home/ubuntu/ros_catkin_ws/devel_isolated/genmsg;/home/ubuntu/ros_catkin_ws/devel_isolated/catkin;/home/ubuntu/ros_catkin_ws/devel_isolated/xacro;/home/ubuntu/ros_catkin_ws/install_isolated'.split(';')
        else:
            # don't consider any other prefix path than this one
            CMAKE_PREFIX_PATH = []
        # prepend current workspace if not already part of CPP
        base_path = os.path.dirname(__file__)
        # CMAKE_PREFIX_PATH uses forward slash on all platforms, but __file__ is platform dependent
        # base_path on Windows contains backward slashes, need to be converted to forward slashes before comparison
        if os.path.sep != '/':
            base_path = base_path.replace(os.path.sep, '/')

        if base_path not in CMAKE_PREFIX_PATH:
            CMAKE_PREFIX_PATH.insert(0, base_path)
        CMAKE_PREFIX_PATH = os.pathsep.join(CMAKE_PREFIX_PATH)

        environ = dict(os.environ)
        lines = []
        if not args.extend:
            lines += rollback_env_variables(environ, ENV_VAR_SUBFOLDERS)
        lines += prepend_env_variables(environ, ENV_VAR_SUBFOLDERS, CMAKE_PREFIX_PATH)
        lines += find_env_hooks(environ, CMAKE_PREFIX_PATH)
        print('\n'.join(lines))

        # need to explicitly flush the output
        sys.stdout.flush()
    except IOError as e:
        # and catch potential "broken pipe" if stdout is not writable
        # which can happen when piping the output to a file but the disk is full
        if e.errno == errno.EPIPE:
            print(e, file=sys.stderr)
            sys.exit(2)
        raise

    sys.exit(0)
