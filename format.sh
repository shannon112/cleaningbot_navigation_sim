#!/bin/bash

# Set the path to clang-format-9
CLANG_FORMAT=clang-format-9

# Find all .cpp and .h files in include and src directories and format them
find include src test -name "*.cpp" -o -name "*.h" -o -name "*.hpp" | while read -r file; do
    echo "Formatting $file"
    $CLANG_FORMAT -i "$file"
done

echo "All files formatted!"
