# Copyright (c) 2018 CNRS
# Authors: Joseph Mirabel
#
# This file is part of sot-gepetto-viewer
# sot-gepetto-viewer is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# sot-gepetto-viewer is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-gepetto-viewer  If not, see
# <http://www.gnu.org/licenses/>.

INSTALL(
  FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/__init__.py
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/callback_after_robot_increment.py
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/command_execution.py
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/graph.py
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/plot.py
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/ros_init_node.py
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/sot_local_plugin.py
  ${CMAKE_CURRENT_SOURCE_DIR}/sot_gepetto_viewer/sot_plugin.py
  DESTINATION ${PYTHON_SITELIB}/sot_gepetto_viewer
  )
