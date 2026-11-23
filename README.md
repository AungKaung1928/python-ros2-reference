# Python Fundamentals for ROS2 Robotics
Quick reference guide for Python syntax and ROS2 examples on Ubuntu 22.04 with ROS2 Humble.

## Run a lesson
Every file imports `rclpy`, so source ROS 2 Humble first.
```bash
git clone https://github.com/AungKaung1928/python-ros2-reference.git
cd python-ros2-reference
source /opt/ros/humble/setup.bash

cd 07_functions
python3 -c "import functions; functions.general_example()"   # plain-Python part, prints to the terminal
python3 functions.py   # ROS 2 part: starts the node; it waits for /scan (e.g. TurtleBot3 in Gazebo)
```
Lessons 03, 04, 05, 06 and 15 ask for keyboard input in the plain-Python part.

##  Contents

### Basics
- [01 - Hello World](./01_helloworld) - Basic output and program structure
- [02 - Data Types](./02_datatypes) - int, float, str, bool and type operations
- [03 - Input/Output](./03_inputoutput) - input(), print(), and basic I/O
- [04 - If/Else](./04_ifelse) - Conditional statements and decision making
- [05 - Loops](./05_loops) - for, while, and iteration patterns
- [06 - Break/Continue](./06_breakcontinue) - Loop control flow
- [07 - Functions](./07_functions) - Function definition, arguments, and return values

### Data Structures
- [08 - Lists](./08_lists) - Mutable sequences, indexing, slicing, methods
- [09 - Tuples](./09_tuples) - Immutable sequences and unpacking
- [10 - Dictionaries](./10_dictionaries) - Key-value pairs and hash maps
- [11 - Sets](./11_sets) - Unique collections and set operations
- [12 - List Comprehension](./12_list_comprehension) - Concise list creation patterns

### Advanced
- [13 - Classes](./13_classes) - OOP basics, __init__, methods
- [14 - Inheritance](./14_inheritance) - Base and derived classes, super()
- [15 - Exception Handling](./15_exception_handling) - try/except/finally blocks
- [16 - File Handling](./16_file_handling) - Reading/writing files with context managers
- [17 - Lambda Functions](./17_lambda_functions) - Anonymous functions (Critical for ROS2)
- [18 - Decorators](./18_decorators) - Function wrappers and metaprogramming
- [19 - Generators](./19_generators) - yield and memory-efficient iteration

##  Purpose
Each file contains:
1. **General Python Example** - Pure Python syntax reference
2. **ROS2 Example** - Practical robotics application

##  Notes
- Python 3.10+, ROS 2 Humble.
- Lidar examples pick front/left/right with `ray_index(msg, angle)` (0 = ahead, +pi/2 = left), so they work on a 0..2pi scan (TurtleBot3) and a -pi..pi scan.
- Lambda functions and classes are **essential** for ROS2 Python.

## Tests
`python3 -m pytest -q tests` (ROS 2 sourced) runs every lesson's plain-Python part and checks `ray_index` on both scan layouts. CI runs the same in a `ros:humble` container.

##  Learning Path
**Beginners**: 01 → 07  
**Intermediate**: 08 → 12  
**Advanced/ROS2 Ready**: 13 → 19

## License

MIT, see LICENSE.
