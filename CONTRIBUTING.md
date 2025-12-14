# Contributing

## Commits
- Write clear, imperative commit messages (short summary on first line).
- Aim for a 50-character summary, optional longer description after a blank line.
- One logical change per commit.

## Pull requests
- Use descriptive PR titles.
- Include a short description of what changed and how to test it.

## Code style & naming

### Files & Folders

| Item               | Convention               | Example / Notes                 |
|--------------------|--------------------------|---------------------------------|
| File names         | snake_case               | `data_processor.py`, `utils.c`  |
| Folder names       | snake_case               | `data_files/`, `config_scripts/`|

### Python

| Item                   | Convention                       | Example / Notes                       |
|------------------------|----------------------------------|---------------------------------------|
| Files & modules        | snake_case.py                    | `my_module.py`                        |
| Functions & variables  | snake_case                       | `def calculate_total():`              |
| Classes                | PascalCase (CapWords)            | `class DataProcessor:`                |
| Constants              | UPPER_SNAKE_CASE                 | `MAX_RETRIES = 5`                     |
| Function size & naming | Keep functions small (single responsibility); descriptive names | Break complex logic into helpers; use clear names |

### C

| Item                       | Convention                    | Example / Notes                                                   |
|----------------------------|-------------------------------|-------------------------------------------------------------------|
| Files & modules            | snake_case.c                  | `device_driver.c`                                                 |
| Header files               | Match basename with `.c` file | `device_driver.h` for `device_driver.c`                           |
| Functions                  | snake_case                    | `void init_device(void);`                                         |
| Variables                  | snake_case                    | `int buffer_size;`                                                |
| Types (typedefs, structs, enums) | PascalCase              | `typedef struct DeviceConfig { ... } DeviceConfig;`               |
| Macros & constants         | UPPER_SNAKE_CASE              | `#define MAX_SIZE 256` or `const int MAX_SIZE = 256;`             |
| Enum values                | UPPER_SNAKE_CASE              | `typedef enum { DEVICE_STATE_OFF, DEVICE_STATE_ON } DeviceState;` |
| Header include guards      | UPPER_SNAKE_CASE              | `PROJECT_NETWORK_CONFIG_H`                                        |
| Formatting                 | 4 spaces for indentation (no tabs); keep functions short | Consistent style across files          |
| Pointer placement          | Prefer `type *ptr`            | `char *name;`                                                     |
