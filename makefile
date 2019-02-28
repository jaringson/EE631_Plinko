all: basic


basic : basic.cpp
	g++ basic.cpp -o $@ -std=c++14 `pkg-config --cflags --libs opencv`

clean :
	-rm basic