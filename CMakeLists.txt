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

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME "sot-gepetto-viewer")
# SET(PROJECT_URL "")
SET(PROJECT_DESCRIPTION "Plugins for SoT inside Gepetto viewer")

FINDPYTHON(2.7 REQUIRED)

ADD_SUBDIRECTORY(src)
