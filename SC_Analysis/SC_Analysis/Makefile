.PHONY: clean prep

all: prep anal

prep: 
	@mkdir -p build

point:
	g++ -Wall -c Point.cpp -o  build/Point.o

frame:
	g++ -Wall -c Frame.cpp -o  build/Frame.o

helpers:
	g++ -Wall -c helpers.cpp -o  build/helpers.o

front:
	g++ -Wall -c front.cpp -o  build/front.o


anal: point frame helpers front
	g++ -Wall -D _DEBUG build/{Point,Frame,helpers,front}.o SC_Analysis.cpp -o build/sc

test: prep point frame helpers front
	g++ -Wall build/{Point,Frame,helpers,front}.o ../SC_Tests/main.cpp -o build/tests;

clean:
	@rm -rf build/*.o build/sc build/
