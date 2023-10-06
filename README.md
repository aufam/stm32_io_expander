# I/O exapander interface for STM32

## Requirements
* C++17
* cmake minimum version 3.7
* STM32CubeMx generated code

## How to use
* Clone this repo to your STM32 project folder. For example:
```bash
git clone https://github.com/aufam/stm32_io_expander.git your_project_path/Middlewares/Third_Party/stm32_io_expander
```
* Add these line to your project CMakeLists.txt:
```cmake
add_subdirectory(Middlewares/Third_Party/stm32_io_expander)
target_link_libraries(${PROJECT_NAME}.elf io_expander)
```
* (Optional) Add the submodule:
```bash
git submodule add https://github.com/aufam/stm32_io_expander.git your_project_path/Middlewares/Third_Party/stm32_io_expander
```
