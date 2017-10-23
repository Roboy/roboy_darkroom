#
#    Copyright 2016 Rethink Robotics
#
#    Copyright 2016 Chris Smith
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

## ROS message source code generation for JS
##
## Converts ROS .msg files in a package into JS source files

import gennodejs
import sys

if __name__ == "__main__":
    gennodejs.gennodejs_main.genmain(sys.argv, 'gen_nodejs.py')
