::set JAVA_HOME=c:\Program Files\Java\jdk-14.0.2
set JAVAC=%JAVA_HOME%\bin\javac.exe

set CLASS_DIR="classes"
set SRC_DIR="src"

dir /s /B %SRC_DIR%\*.java > build_sources.txt
"%JAVAC%" -d %CLASS_DIR% @build_sources.txt
pause