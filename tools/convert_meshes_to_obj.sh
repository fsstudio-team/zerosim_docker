#!/bin/bash
# Recursively converts all .DAE & .STL files to .OBJ using blender.
# Usage: ./convert_meshes_to_obj.sh directoy/of/meshes/

DIRECTORY=$1

DAE_FILES=$(find "${DIRECTORY}" -name '*.dae')

for f in $DAE_FILES; do
	blender -b -P /zerosim_tools/dae_to_obj.py  -- "$f" "${f%.dae}.obj"
done
	
STL_FILES=$(find "${DIRECTORY}" -name '*.stl')
for f in $STL_FILES; do
	blender -b -P /zerosim_tools/stl_to_obj.py  --  "$f" "${f%.stl}.obj"
done

