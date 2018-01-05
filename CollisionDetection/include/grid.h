#include <vector>
#include "coreMath.h"
#include "particle.h"

struct Cell {


	// Vector stores all particles contained in this cell
	std::vector<Particle*> occupants;
};

class Grid {
private:
	std::vector<Cell> cells;

public:
	Grid(int width, int height, int cellSize);
	~Grid();
	// Retrieve the cell corresponding to given co-ordinates
	Cell* getCell(float x, float y);

};