@echo off
rem (build\x64-Release\TrimeshTracer.exe 640 360 4 data/triangle.obj) && (move /Y output.png output1Triangle.png)
(build\x64-Release\TrimeshTracer.exe 640 360 4 data/cube.obj) && (move /Y output.png output2Cube.png)
(build\x64-Release\TrimeshTracer.exe 640 360 4 data/suzanne.obj) && (move /Y output.png output3Suzanne.png)
(build\x64-Release\TrimeshTracer.exe 640 360 4 data/teapot.obj) && (move /Y output.png output4Teapot.png)
rem (build\x64-Release\TrimeshTracer.exe 640 360 4 data/sponza.obj) && (move /Y output.png output5Sponza.png)
pause
