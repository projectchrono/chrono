## =============================================================================
## PROJECT CHRONO - http://projectchrono.org
##
## Copyright (c) 2020 projectchrono.org
## All rights reserved.
##
## Use of this source code is governed by a BSD-style license that can be found
## in the LICENSE file at the top level of the distribution and at
## http://projectchrono.org/license-chrono.txt.
##
## =============================================================================
## Authors: Colin Vanden Heuvel
## =============================================================================
##
## Further Notes:
##
## Emulates the behavior of embedfile by Hammad Mazhar
## http://hamelot.io/visualization/opengl-glsl-shader-as-a-string/
##
## embedfile was in turn based on txt2h by ScottManDeath
## https://community.khronos.org/t/glsl-shader-embedded-in-c/53110
##
## =============================================================================

# embedfile.cmake

# PARAMETERS:
#	SYMBOL - A valid C/C++ symbol name to identify the embedded data
#	DATA_FILE - The path to an input data file (in ASCII format) which will be
#				embedded into the generated header
#	HEADER_FILE - The path to a C or C++ output header file which will contain
#				the encoded data

# Read data file into variable, then process it into a usable C string
FILE(READ "${DATA_FILE}" raw_data)
STRING(REPLACE "\\" "\\\\" pass_1 "${raw_data}")
STRING(REPLACE "\"" "\\\"" pass_2 "${pass_1}")
STRING(REPLACE "\n" "\\n\"\n\"" escaped_data "${pass_2}")

# Write opening lines for header
FILE(WRITE "${HEADER_FILE}" "// Header generated from Project Chrono data files\n")
FILE(APPEND "${HEADER_FILE}" "#ifndef TXT_HEADER_${SYMBOL}\n")
FILE(APPEND "${HEADER_FILE}" "#define TXT_HEADER_${SYMBOL}\n")
FILE(APPEND "${HEADER_FILE}" "const char ${SYMBOL} [] =\n\"")

# Write transformed data file
FILE(APPEND "${HEADER_FILE}" "${escaped_data}")
FILE(APPEND "${HEADER_FILE}" "\"")

# Write closing lines
FILE(APPEND "${HEADER_FILE}" ";\n")
FILE(APPEND "${HEADER_FILE}" "#endif // TXT_HEADER_${SYMBOL}\n")
