#!/usr/bin/env bash

set -euo pipefail

# sudo apt install clang-format
find include/ -regex '.*\.\(cc\|cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find src/ -regex '.*\.\(cc\|cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find app/ -regex '.*\.\(cc\|cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find tests/ -regex '.*\.\(cc\|cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
