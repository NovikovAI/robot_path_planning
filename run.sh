/usr/bin/cmake --no-warn-unused-cli -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc-12 -S/home/user/Desktop/robot_path_planning -B/home/user/Desktop/robot_path_planning/build -G "Unix Makefiles"
cd /home/user/Desktop/robot_path_planning/build/
make
./main
