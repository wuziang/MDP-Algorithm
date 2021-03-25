package map;

/**
 * Represents each cell in the map grid.
 */

public class Cell {
    private final int row;
    private final int col;

    private boolean isVirtualWall;
    private boolean isObstacle;
    private boolean isExplored;

    private boolean isVisited;
    private int positionCount;

    private boolean[] isProcessed;

    public Cell(int row, int col) {
        this.row = row;
        this.col = col;
        this.isProcessed = new boolean[4];
    }

    public int getRow() {
        return this.row;
    }

    public int getCol() {
        return this.col;
    }

    public void setIsObstacle(boolean val) { this.isObstacle = val; }

    public boolean getIsObstacle() {
        return this.isObstacle;
    }

    public void setVirtualWall(boolean val) {
        if (val) {
            this.isVirtualWall = true;
        } else {
            if (row != 0 && row != MapConstants.MAP_ROWS - 1 && col != 0 && col != MapConstants.MAP_COLS - 1) {
                this.isVirtualWall = false;
            }
        }
    }

    public boolean getIsVirtualWall() {
        return this.isVirtualWall;
    }

    public void setIsExplored(boolean val) {
        this.isExplored = val;
    }

    public boolean getIsExplored() {
        return this.isExplored;
    }

    public void setIsVisited(boolean val) { this.isVisited = val; }

    public boolean getIsVisited() { return this.isVisited; }

    public void setPositionCount() { this.positionCount++; }

    public void clearPositionCount() { this.positionCount =0; }

    public int getPositionCount() { return this.positionCount; }

    public void setIsProcessed(int index, boolean val) {
        this.isProcessed[index] = val;
    }

    public boolean getIsProcessed(int index) {
        return this.isProcessed[index];
    }
}
