build:
	g++ -g -std=c++17 -Wall ./src/*.cpp ./src/physics/*.cpp ./src/physics/shape/*.cpp -lm -lSDL2 -lSDL2_image -lSDL2_gfx -o app.out

run:
	./app.out

clean:
	rm app.out

