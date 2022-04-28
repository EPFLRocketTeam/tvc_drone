#!/bin/bash

if [[ $# -ne 1 ]]; then
    echo "Please specify the new package name" >&2
    exit 2
fi

packageName=$1
packageName="${packageName,,}"

TEMPLATE_GNC_PATH=$(realpath $(dirname $0)/..)

rm -rf "$TEMPLATE_GNC_PATH"/.git

# rename file contents
find "$TEMPLATE_GNC_PATH" -type f -exec sed -i -e 's/pid_/'"$packageName"'_/g' {} \;
find "$TEMPLATE_GNC_PATH" -type f -exec sed -i -e 's/Pid/'"${packageName^}"'/g' {} \;

#rename directories and filenames
for depth in $(seq 0 4);
do
    find "$TEMPLATE_GNC_PATH"/* -maxdepth $depth -name "*template*" | while read -r file; do
        cd $(dirname $file)
        filename=$(basename $file)
        mv "$filename" "${filename//template/$packageName}"
    done
done

mv "$TEMPLATE_GNC_PATH" "${TEMPLATE_GNC_PATH//template/$packageName}"

echo "Updated name of package to $packageName""_gnc"