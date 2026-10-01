// navigation.cpp
// Ported from multicore_version (M7_main/navigation.ino) into the single-core
// main branch. All RPC.call(...) cross-core calls have been replaced with the
// original single-core functions (drivetrain.encoderCountA, detectWall, ...).
// Adds the multi-floor elevation features (m1/m2/m3, elevation/descend) and the
// encoder-based victim tile marking, while keeping main's working BFS.

#include "Globals.h"
#include <ArduinoQueue.h>
#include <deque>

Direction rotateDir(Direction base, int offset) {
  return (Direction)((base + offset + 4) % 4);
}
// at orientation w, conver heading of x to local heading of y.

void stepForward(Direction d, int &x, int &y) {
  // global direction
  if (d == NORTH) y++;
  else if (d == EAST) x++;
  else if (d == SOUTH) y--;
  else if (d == WEST) x--;
}
// pulses for distance(mm)
double pulsesForDistanceMm(double distanceMm) {
  return distanceMm / (wheel_diameter * M_PI) * wheel_cpr * gear_ratio;
}
// which tile victim is depending on encoder.
void victimTileFromEncoder(int distanceMm, int encoderCount, int &victimX, int &victimY) {
  victimX = x_pos;
  victimY = y_pos;

  double tilePulses = pulsesForDistanceMm(distanceMm); // total pulses to traverse a tile.
  if(abs(encoderCount) >= tilePulses / 2.0){
    stepForward(currentDir, victimX, victimY); // past tile midpoint -> victim belongs to the next tile.
  }
}
// mark victim of tile based on encoder position (single-core: read encoder directly)
void markVictimAtEncoderPosition(int distanceMm) {
  int encoderCount = (drivetrain.encoderCountA+drivetrain.encoderCountB+drivetrain.encoderCountD)/3;
  int victimX, victimY;
  victimTileFromEncoder(distanceMm, encoderCount, victimX, victimY);
  if(!inBounds(victimX, victimY)) return;

  mapGrid[victimX][victimY].setVictim(true);
  //mapGrid[victimX][victimY].setDiscovered(true);

  Serial.print("marked victim at tile x=");
  Serial.print(victimX);
  Serial.print(", y=");
  Serial.print(victimY);
  Serial.print(", encoderA=");
  Serial.println(encoderCount);
}

bool inBounds(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

void initializeMap() {
  for (int x = 0; x < MAP_SIZE; x++) {
    for (int y = 0; y < MAP_SIZE; y++) {
      mapGrid[x][y].setDiscovered(false);
      mapGrid[x][y].setFully(false);
      mapGrid[x][y].setVisited(false);
      mapGrid[x][y].setElevate(false);
      mapGrid[x][y].setDescend(false);
      for (int d = 0; d < 4; d++) {
        mapGrid[x][y].setWall(d, false);
        mapGrid[x][y].setEdge(d, false);
        mapGrid[x][y].setObstacle(d, false);
      }
      mapGrid[x][y].setType(BLANK);
    }
  }
}

// mark traveled edge in BOTH tiles (current and next)
void markEdgeBothWays(int x, int y, Direction d) {
  int nx = x, ny = y;
  stepForward(d, nx, ny);
  if (!inBounds(nx, ny)) return;

  mapGrid[x][y].setEdge(d, true); // connected
  mapGrid[nx][ny].setEdge(opposite(d), true); // update both sides.
}

// update fullyExplored = all OPEN dirs have been traveled at least once
// necessary for picknextdirection.
void updateFullyExploredAt(int x, int y) {
  Tile &t = mapGrid[x][y];
  bool allDone = true;
  t.setVisited(true);
  for (int d = 0; d < 4; d++) {
    if (t.getWall(d) == false) {     // open
      if (t.getEdge(d) == false) {   // not traveled yet
        allDone = false;
        break;
      }
    }
  }
  t.setFully(allDone);
}
// 0=front, 1=right, 2=back, 3=left
void readWallsRel(bool &wallF, bool &wallR, bool &wallB, bool &wallL) { // references needed here to update the variable values
  wallF = (detectWall(0)==0);
  wallR = (detectWall(1)==0);
  wallB = (detectWall(2)==0);
  wallL = (detectWall(3)==0);
  Serial.println(wallF);
  Serial.println(wallR);
  Serial.println(wallB);
  Serial.println(wallL);
}
//get the wall from L,R(local) into N W(global)
// absF is the absolute heading the the robot front is heading.
void writeWallsToCurrentTile(bool wallF, bool wallR, bool wallB, bool wallL) {
  Tile &t = mapGrid[x_pos][y_pos];
  t.setDiscovered(true); // tile is discovered(walls are found)
  // shouldn't absolute directions be north south east west?
  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  Direction absL = rotateDir(currentDir, -1);
  // An edge the robot has already driven through can't be a wall, so a reading
  // that says it is is a misread (usually the wall behind the robot right after it
  // arrived). Storing it could cut off the only route home in BFS().
  t.setWall(absF, wallF && !t.getEdge(absF));
  t.setWall(absR, wallR && !t.getEdge(absR));
  t.setWall(absB, wallB && !t.getEdge(absB));
  t.setWall(absL, wallL && !t.getEdge(absL));
  // need to mark both ways.
}
// FIXME: this still needs to be fixed - the re-sense / position-mismatch logic
// is not trustworthy yet and should be revisited before being relied on.
// Re-sense check: does the freshly-sensed wall pattern at the current tile agree
// with what the map already recorded for it? Only meaningful once the tile has
// actually been visited before (getVisited(), not getDiscovered() -- the home
// tile is marked discovered in setup() before any real walls are ever sensed,
// so gating on getDiscovered() would false-positive "mismatch" on the very
// first tile at power-on). Tolerates a single disagreeing wall (sensor noise)
// via WALL_MISMATCH_THRESHOLD before flagging the position as unreliable.
bool checkTileMismatch(bool wallF, bool wallR, bool wallB, bool wallL) {
  Tile &t = mapGrid[x_pos][y_pos];
  if (!t.getVisited()) return false; // no trustworthy prior data for this tile yet

  Direction absF = currentDir;
  Direction absR = rotateDir(currentDir, +1);
  Direction absB = rotateDir(currentDir, +2);
  Direction absL = rotateDir(currentDir, -1);

  int mismatches = 0;
  if (t.getWall(absF) != wallF) mismatches++;
  if (t.getWall(absR) != wallR) mismatches++;
  if (t.getWall(absB) != wallB) mismatches++;
  if (t.getWall(absL) != wallL) mismatches++;

  return mismatches >= WALL_MISMATCH_THRESHOLD;
}

int turnNeededDeg(int direction) {
  // Convert an absolute direction enum to an absolute heading angle.
  if (direction == 0) return 0;
  if (direction == 1) return 90;
  if (direction == 2) return 180;
  return 270; // diff==3

}
int dir[4][2] = {
    {0, 1},
    {1, 0},
    {0, -1},
    {-1, 0}
};
void initTile(int x, int y, Grid& map) { //needs update (probably unneeded, small prio)
    map[x][y].setDiscovered(false);
    map[x][y].setFully(false);
    map[x][y].setVisited(false);
    map[x][y].setElevate(false);
    map[x][y].setDescend(false);
    for (int d = 0; d < 4; d++) {
        map[x][y].setWall(d, false);
        map[x][y].setEdge(d, false);
        map[x][y].setObstacle(d, false);
    }
    map[x][y].setType(BLANK);
}

void reallocate(Grid& mapgrid, int pos_x = 0, int pos_y = 0) { //input mapgrid, and next tile location
    //cout << "start reallocate" << endl;

    //expand to bottom (remove top)
    if (pos_y >= MAP_SIZE) {
        for (int i = 0; i < MAP_SIZE - 1; i++) {
            for (int j = 0; j < MAP_SIZE; j++) {
                mapgrid[i][j] = mapgrid[i + 1][j];
                m1[i][j] = m1[i + 1][j];
                m2[i][j] = m2[i + 1][j];
                m3[i][j] = m3[i + 1][j];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(MAP_SIZE - 1, i, mapgrid);
            initTile(MAP_SIZE - 1, i, m1);
            initTile(MAP_SIZE - 1, i, m2);
            initTile(MAP_SIZE - 1, i, m3);
        }
    }
    //expand to top (remove bottom)
    else if (pos_y < 0) {
        for (int i = MAP_SIZE-1; i > 0; i--) {
            for (int j = 0; j < MAP_SIZE; j++) {
                mapgrid[i][j] = mapgrid[i - 1][j];
                m1[i][j] = m1[i - 1][j];
                m2[i][j] = m2[i - 1][j];
                m3[i][j] = m3[i - 1][j];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(0, i, mapgrid);
            initTile(0, i, m1);
            initTile(0, i, m2);
            initTile(0, i, m3);
        }
    }

    //expand to right (remove left)
    else if (pos_x >= MAP_SIZE) {
        for (int j = 0; j < MAP_SIZE - 1; j++) {
            for (int i = 0; i < MAP_SIZE; i++) {
                mapgrid[i][j] = mapgrid[i][j+1];
                m1[i][j] = m1[i][j+1];
                m2[i][j] = m2[i][j+1];
                m3[i][j] = m3[i][j+1];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(i, MAP_SIZE-1, mapgrid);
            initTile(i, MAP_SIZE-1, m1);
            initTile(i, MAP_SIZE-1, m2);
            initTile(i, MAP_SIZE-1, m3);
        }
    }

    //expand to left (remove right)
    else if (pos_x < 0) {
        for (int j = MAP_SIZE - 1; j > 0; j--) {
            for (int i = 0; i < MAP_SIZE; i++) {
                mapgrid[i][j] =  mapgrid[i][j - 1];
                m1[i][j] = m1[i][j-1];
                m2[i][j] = m2[i][j-1];
                m3[i][j] = m3[i][j-1];
            }
            //printmap(mapgrid);
        }
        for (int i = 0; i < MAP_SIZE; i++) {
            initTile(i, 0, mapgrid);
            initTile(i, 0, m1);
            initTile(i, 0, m2);
            initTile(i, 0, m3);
        }
    }
}

// floor index -> its stored grid (m1 = floor 0, m2 = floor 1, m3 = floor 2)
Grid& floorGrid(int floor) {
  if (floor <= 0) return m1;
  if (floor == 1) return m2;
  return m3;
}

// mapGrid is a working copy of the current floor: write it back into its floor
// slot so BFS (which reads m1/m2/m3) sees the latest walls/visits.
void syncActiveFloor() {
  floorGrid(currentFloor) = mapGrid;
}

// Record the ramp tile the robot is standing on (open front/back, walls at sides).
static void recordRampTile() {
  markEdgeBothWays(x_pos, y_pos, currentDir);
  writeWallsToCurrentTile(0, 1, 0, 1);
  updateFullyExploredAt(x_pos, y_pos);
}

// Move up one floor. The current floor grid is saved into its storage grid and
// the active mapgrid is swapped to the floor above. The lower copy of the ramp
// tile is flagged elevate and the upper copy descend, which is how BFS links floors.
// (m1/m2/m3 params kept for the existing call sites; floorGrid() uses the same globals.)
void elevation(Grid& mapgrid, int xpos, int ypos, Grid& m1, Grid& m2, Grid& m3, int& floor){
  if (floor + 1 >= NUM_FLOORS) {
    // no stored floor above: keep mapping on this grid rather than indexing past m3
    Serial.println("elevation: no floor slot above, staying on this grid");
    recordRampTile();
    return;
  }
  mapgrid[xpos][ypos].setElevate(true);
  mapgrid[xpos][ypos].setRampUp(currentDir); // driving uphill
  floorGrid(floor) = mapgrid;
  floor++;
  mapgrid = floorGrid(floor);
  mapgrid[xpos][ypos].setDescend(true); // bidirection elevate/descend
  mapgrid[xpos][ypos].setRampUp(currentDir);
  recordRampTile();
}

// Move down one floor.
void descend(Grid& mapgrid, int xpos, int ypos, Grid& m1, Grid& m2, Grid& m3, int& floor){
  if (floor - 1 < 0) {
    // no stored floor below: keep mapping on this grid (the old code reloaded m1
    // here, which threw away everything mapped on floor 0 since the last save)
    Serial.println("descend: no floor slot below, staying on this grid");
    recordRampTile();
    return;
  }
  mapgrid[xpos][ypos].setDescend(true);
  mapgrid[xpos][ypos].setRampUp(opposite(currentDir)); // driving downhill
  floorGrid(floor) = mapgrid;
  floor--;
  mapgrid = floorGrid(floor);
  mapgrid[xpos][ypos].setElevate(true);
  mapgrid[xpos][ypos].setRampUp(opposite(currentDir));
  recordRampTile();
}

// Floor of the neighbour reached by moving from (z,x,y) in direction d.
// A ramp tile is stored on both floors (lower copy flagged elevate, upper copy
// descend). Stepping ONTO it switches to the other floor's copy; stepping OFF it
// goes uphill -> upper floor, downhill -> lower floor. Without the second rule,
// leaving a ramp copy toward the end it was entered from looked up a tile on the
// wrong floor that was never visited, so the planner kept driving back over the
// ramp to "explore" it.
int neighbourFloor(int z, int x, int y, int d, int nx, int ny) {
  Tile &cur = floorGrid(z)[x][y];
  if (cur.getElevate() || cur.getDescend()) {
    int upper = cur.getElevate() ? z + 1 : z;
    int lower = upper - 1;
    Direction up = cur.getRampUp();
    if (d == up && upper < NUM_FLOORS) return upper;
    if (d == opposite(up) && lower >= 0) return lower;
    return z;
  }
  Tile &probe = floorGrid(z)[nx][ny];
  if (probe.getElevate() && z + 1 < NUM_FLOORS) return z + 1;
  if (probe.getDescend() && z - 1 >= 0) return z - 1;
  return z;
}

// old 2d bfs
// pair structure
/*
int BFS(coord currentpos, Grid& mapGrid, coord endpos, coord path[MAP_SIZE * MAP_SIZE]) { // auto updates path
    ArduinoQueue<coord> queue = {};
    size_t rows = MAP_SIZE;
    size_t columns = MAP_SIZE;
    bool visited[MAP_SIZE][MAP_SIZE] = {false};
    coord prev[MAP_SIZE][MAP_SIZE];
    queue.enqueue(currentpos); // current tile
    visited[currentpos.x][currentpos.y] = true;
    //search
    while (queue.itemCount() > 0) {
        int x = queue.getHead().x; int y = queue.getHead().y;
        //cout << "visting: " << x << "," << y << endl;

        for (int i = 0; i < 4; i++) {
            int nx = x + dir[i][0];
            int ny = y + dir[i][1];
            if (nx < rows && ny < columns && nx >= 0 && ny >= 0) {
                if (!visited[nx][ny] &&
                    !mapGrid[x][y].getWall((Direction)i) &&
                    !mapGrid[nx][ny].getWall(opposite((Direction)i)) &&
                    mapGrid[nx][ny].getDiscovered() &&
                    mapGrid[nx][ny].getType() != BLACK) { //IMPORTANT: ADD MORE CONDITIONALS HERE
                    queue.enqueue(coord{nx, ny}); // add tile
                    visited[nx][ny] = true;

                    prev[nx][ny] = coord{x, y};

                }
            }
        }
        queue.dequeue();
    }
    // Check endpos was actually reached before reconstructing
    if (!visited[endpos.x][endpos.y]) {
      return 0; // endpos unreachable — caller must handle empty path
    }
    //reconstruct path
    // path[0] is endpos, path[n] is current tile.
    int i = 0;
    coord curr = endpos;
    while (true) {
      path[i] = curr;
      i++;

      if (curr.x == currentpos.x&&curr.y==currentpos.y) {
        break;
      }
      curr = prev[curr.x][curr.y];
    }
    return i;

}
*/
// Compact BFS node. uint8_t is safe because MAP_SIZE (40) and floors (3) both
// fit easily; keeps the static scratch arrays small.
struct BfsNode { uint8_t z, x, y; }; // 3d coords in form z (floor) ,x,y

// allowBlue: if true, BLUE tiles are traversable (fallback mode).
// Returns empty deque if endpos is unreachable under the given constraints.
std::deque<std::pair<int, std::pair<int,int>>> BFS(std::pair<int, std::pair<int, int>> currentpos, Grid& m1, Grid& m2, Grid& m3, std::pair<int, std::pair<int, int>> endpos, bool allowBlue, bool allowObstacle) {
    Grid* map[3] = { &m1, &m2, &m3 };  // index, don't copy

    static bool    visited[3][MAP_SIZE][MAP_SIZE];
    static BfsNode prev[3][MAP_SIZE][MAP_SIZE];
    static BfsNode queue[3 * MAP_SIZE * MAP_SIZE]; // each node enqueued once -> never overflows
    memset(visited, 0, sizeof(visited));
    int head = 0, tail = 0;

    BfsNode start = { (uint8_t)currentpos.first, (uint8_t)currentpos.second.first, (uint8_t)currentpos.second.second };
    int ez = endpos.first, ex = endpos.second.first, ey = endpos.second.second;

    // already at the goal: return a trivial path. Guards the reconstruction loop
    if (start.z == ez && start.x == ex && start.y == ey) {
        return { currentpos };
    }

    queue[tail++] = start;
    visited[start.z][start.x][start.y] = true;
    while (head < tail) {
        BfsNode cur = queue[head++];
        int x = cur.x, y = cur.y, z = cur.z;

        for (int i = 0; i < 4; i++) {
            int nx = x + dir[i][0];
            int ny = y + dir[i][1];

            if (nx >= 0 && nx < MAP_SIZE && ny >= 0 && ny < MAP_SIZE) {
                // floor change across ramp tiles (same rule as the exploration planner)
                int nz = neighbourFloor(z, x, y, i, nx, ny);

                bool passable = !(*map[z])[x][y].getWall((Direction)i) &&
                                !(*map[nz])[nx][ny].getWall(opposite((Direction)i)) &&
                                (*map[nz])[nx][ny].getDiscovered() &&
                                (*map[nz])[nx][ny].getType() != BLACK;
                if (!allowBlue) {
                    passable = passable && (*map[nz])[nx][ny].getType() != BLUE;
                }
                if (!allowObstacle){
                    // obstacle bits are per edge (set on both tiles by EXECUTE_MOVE /
                    // handleShortMove), so only this edge is blocked, not the whole tile
                    passable = passable && !(*map[z])[x][y].getObstacle(i) &&
                               !(*map[nz])[nx][ny].getObstacle(opposite((Direction)i));
                }

                if (!visited[nz][nx][ny] && passable) {
                    visited[nz][nx][ny] = true;
                    prev[nz][nx][ny] = cur;
                    queue[tail++] = { (uint8_t)nz, (uint8_t)nx, (uint8_t)ny };
                }
            }
        }
    }

    // endpos unreachable under current constraints >> return empty path
    if (!visited[ez][ex][ey]) {
        return {};
    }

    // reconstruct: walk backward from endpos to start via prev[], push_front
    // so path[0]=currentpos, path[last]=endpos
    std::deque<std::pair<int, std::pair<int,int>>> path;
    BfsNode curr = { (uint8_t)ez, (uint8_t)ex, (uint8_t)ey };
    while (!(curr.z == start.z && curr.x == start.x && curr.y == start.y)) {
        path.push_front({ curr.z, { curr.x, curr.y } });
        curr = prev[curr.z][curr.x][curr.y];
    }
    path.push_front(currentpos);
    return path;
}

// Can the robot drive from (z,x,y) in absolute direction d? No wall on either
// side of the edge, no obstacle recorded on it, target not BLACK. On success
// (nz,nx,ny) is the neighbour, with ramp tiles resolved to their other floor.
static bool edgeTraversable(int z, int x, int y, int d, int &nz, int &nx, int &ny) {
  nx = x + dir[d][0];
  ny = y + dir[d][1];
  if (!inBounds(nx, ny)) return false;
  nz = neighbourFloor(z, x, y, d, nx, ny);

  Tile &t = floorGrid(z)[x][y];
  Tile &n = floorGrid(nz)[nx][ny];
  Direction back = opposite((Direction)d);
  if (t.getWall(d) || t.getObstacle(d)) return false;
  if (n.getWall(back) || n.getObstacle(back)) return false;
  if (n.getType() == BLACK) return false;
  return true;
}

// A visited tile with a traversable edge into a tile that hasn't been visited yet.
static bool isFrontier(int z, int x, int y) {
  for (int d = 0; d < 4; d++) {
    int nz, nx, ny;
    if (edgeTraversable(z, x, y, d, nz, nx, ny) && !floorGrid(nz)[nx][ny].getVisited()) return true;
  }
  return false;
}

// Exploration planner. Returns false when no reachable unexplored tile is left
// (maze fully explored -> go home).
// 1) If an adjacent tile is unvisited, go there (front, right, left, then back -> fewest turns).
// 2) Otherwise BFS through visited tiles (across floors via ramp tiles) to the
//    nearest tile that still has an unvisited neighbour, and take the first step
//    of that path. Re-planned every tile, so no path needs to be stored.
bool planExploreDir(Direction &outDir) {
  syncActiveFloor();
  const int z0 = currentFloor;

  const Direction pref[4] = { currentDir, rotateDir(currentDir, +1), rotateDir(currentDir, -1), rotateDir(currentDir, +2) };
  for (int i = 0; i < 4; i++) {
    int nz, nx, ny;
    if (edgeTraversable(z0, x_pos, y_pos, pref[i], nz, nx, ny) && !floorGrid(nz)[nx][ny].getVisited()) {
      outDir = pref[i];
      return true;
    }
  }

  static bool    seen[NUM_FLOORS][MAP_SIZE][MAP_SIZE];
  static uint8_t firstDir[NUM_FLOORS][MAP_SIZE][MAP_SIZE]; // first move from the robot's tile on the path to this node
  static BfsNode queue[NUM_FLOORS * MAP_SIZE * MAP_SIZE];
  memset(seen, 0, sizeof(seen));
  int head = 0, tail = 0;

  queue[tail++] = { (uint8_t)z0, (uint8_t)x_pos, (uint8_t)y_pos };
  seen[z0][x_pos][y_pos] = true;
  while (head < tail) {
    BfsNode cur = queue[head++];
    bool isStart = (head == 1);
    if (!isStart && isFrontier(cur.z, cur.x, cur.y)) {
      outDir = (Direction)firstDir[cur.z][cur.x][cur.y];
      return true;
    }
    // try the same turn preference so ties favour fewer turns
    for (int i = 0; i < 4; i++) {
      int d = isStart ? pref[i] : i;
      int nz, nx, ny;
      if (!edgeTraversable(cur.z, cur.x, cur.y, d, nz, nx, ny)) continue;
      if (!floorGrid(nz)[nx][ny].getVisited()) continue; // only route through known tiles
      if (seen[nz][nx][ny]) continue;
      seen[nz][nx][ny] = true;
      firstDir[nz][nx][ny] = isStart ? (uint8_t)d : firstDir[cur.z][cur.x][cur.y];
      queue[tail++] = { (uint8_t)nz, (uint8_t)nx, (uint8_t)ny };
    }
  }
  return false; // nothing left to explore
}
