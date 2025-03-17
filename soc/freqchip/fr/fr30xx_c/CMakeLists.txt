# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(${ZEPHYR_BASE}/drivers)
zephyr_sources(
  soc.c
  )

zephyr_include_directories(.)

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/../modules/hal/freqchip/fr30xx_c/components/tools/post_process.py  ${CONFIG_KERNEL_BIN_NAME} ${PROJECT_BINARY_DIR}
)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
