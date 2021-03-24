package map;

/**
 * Represents each cell in the map grid.
 */

public class Cell {
    private final int row;
    private final int col;

    private int isObstacle;
    private boolean isVirtualWall;
    private boolean isExplored;

    private boolean isPledged;

    private double senseCount;
    private double senseObstacle;

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

    public void setIsObstacle(boolean val) {
        if(val) this.senseObstacle++;
        this.senseCount++;
    }

    public boolean getIsObstacle() {
        return (senseObstacle/senseCount)>0.5;
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

    public void setIsPledged(boolean val) { this.isPledged = val; }

    public boolean getIsPledged() { return this.isPledged; }

    public void setIsProcessed(int index, boolean val) {
        this.isProcessed[index] = val;
    }

    public boolean getIsProcessed(int index) {
        return this.isProcessed[index];
    }
}
