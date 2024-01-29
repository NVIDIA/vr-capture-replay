mkdir _build
pushd _build
cmake ..\ -A x64
cmake --build . --config Release
popd