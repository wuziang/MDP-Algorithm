package algorithms;

import map.Cell;
import map.Map;
import map.MapConstants;
import robot.Robot;
import robot.RobotConstants;
import robot.RobotConstants.DIRECTION;
import robot.RobotConstants.MOVEMENT;
import utils.CommMgr;
import utils.MapDescriptor;

import java.util.Random;

/**
 * Exploration algorithm for the robot.
 */

public class ExplorationAlgo {
    private final Map exploredMap;
    private final Map realMap;
    private final Robot bot;

    private int areaExplored;
    private int moves;

    private final int coverageLimit;
    private final int timeLimit;

    private long startTime;
    private long endTime;

    private boolean imageProcessing;
    private boolean imageMode;
    private int takenImage=0;

    private int rightHanded=0;

    public ExplorationAlgo(Map exploredMap, Map realMap, Robot bot, int coverageLimit, int timeLimit) {
        this.exploredMap = exploredMap;
        this.realMap = realMap;
        this.bot = bot;
        this.coverageLimit = coverageLimit;
        this.timeLimit = timeLimit-RobotConstants.STOP_TIME;
    }

    public void setImageProcessing(boolean imageProcessing){
        this.imageProcessing = imageProcessing;
    }

    /**
     * Main method that is called to start the exploration.
     */
    public void runExploration() {
        if(!imageProcessing) {
            System.out.println("\nExploring...");
        }
        else{
            System.out.println("\nImage Processing...");
        }

        if (bot.getRealBot()) {
            CommMgr.getCommMgr().sendMsg("S", CommMgr.AR);
        }

        startTime = System.currentTimeMillis();
        endTime = startTime + (timeLimit * 1000);

        senseAndRepaint();

        // TODO: Android -> PC
//        if (bot.getRealBot()){
//            CommMgr.getCommMgr().recvMsg();
//        }

        explorationLoop(bot.getRobotPosRow(), bot.getRobotPosCol());
    }

    /**
     * Loops through robot movements until one (or more) of the following conditions is met:
     * 1. Robot is back at (r, c)
     * 2. areaExplored > coverageLimit
     * 3. System.currentTimeMillis() > endTime
     */
    private void explorationLoop(int r, int c) {
        do {
            nextMove();
            areaExplored = calculateAreaExplored();

            // This is the stopping condition where r and c are the robot's starting positions
            if (bot.getRobotPosRow() == r && bot.getRobotPosCol() == c) {
                if(areaExplored>=100) {
                    break;
                }
            }
        } while (areaExplored <= coverageLimit && System.currentTimeMillis() <= endTime);

        goHome();

        if(!imageProcessing) {
            areaExplored = calculateAreaExplored();
            System.out.printf("\nExploration Coverage %.2f%%", (areaExplored / 300.0) * 100.0);
            System.out.println("\nTotal Moves: "+moves);

            String[] mapStrings = MapDescriptor.generateMapDescriptor(exploredMap);
            System.out.println("P1: " + mapStrings[0]);
            System.out.println("P2: " + mapStrings[1]);
        }
        else {
            double sides = calculateSidesPossible();
            System.out.printf("\nImage Processing Coverage %.2f%%", (takenImage / sides) * 100.0);
            System.out.println("\nTotal Moves: "+moves);
        }
    }

    /**
     * Determines the next move for the robot and executes it accordingly.
     */
    private void nextMove() {
        if (lookRight()) {
            moveBot(MOVEMENT.RIGHT);
            if (lookForward())
                moveBot(MOVEMENT.FORWARD);
        } else if (lookForward()) {
            moveBot(MOVEMENT.FORWARD);
        } else if (lookLeft()) {
            moveBot(MOVEMENT.LEFT);
            if (lookForward())
                moveBot(MOVEMENT.FORWARD);
        } else {
            moveBot(MOVEMENT.RIGHT);
            moveBot(MOVEMENT.RIGHT);
        }
    }

    /**
     * Returns true if the right side of the robot is free to move into.
     */
    private boolean lookRight() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return eastFree();
            case EAST:
                return southFree();
            case SOUTH:
                return westFree();
            case WEST:
                return northFree();
        }
        return false;
    }

    /**
     * Returns true if the robot is free to move forward.
     */
    private boolean lookForward() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return northFree();
            case EAST:
                return eastFree();
            case SOUTH:
                return southFree();
            case WEST:
                return westFree();
        }
        return false;
    }

    /**
     * * Returns true if the left side of the robot is free to move into.
     */
    private boolean lookLeft() {
        switch (bot.getRobotCurDir()) {
            case NORTH:
                return westFree();
            case EAST:
                return northFree();
            case SOUTH:
                return eastFree();
            case WEST:
                return southFree();
        }
        return false;
    }

    /**
     * Returns true if the robot can move to the north cell.
     */
    private boolean northFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow + 1, botCol - 1) && isExploredAndFree(botRow + 1, botCol) && isExploredNotObstacle(botRow + 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the east cell.
     */
    private boolean eastFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol + 1) && isExploredAndFree(botRow, botCol + 1) && isExploredNotObstacle(botRow + 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the south cell.
     */
    private boolean southFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol - 1) && isExploredAndFree(botRow - 1, botCol) && isExploredNotObstacle(botRow - 1, botCol + 1));
    }

    /**
     * Returns true if the robot can move to the west cell.
     */
    private boolean westFree() {
        int botRow = bot.getRobotPosRow();
        int botCol = bot.getRobotPosCol();
        return (isExploredNotObstacle(botRow - 1, botCol - 1) && isExploredAndFree(botRow, botCol - 1) && isExploredNotObstacle(botRow + 1, botCol - 1));
    }

    /**
     * Returns the robot to START after exploration and points the bot northwards.
     */
    private void goHome() {
        if (!bot.getTouchedGoal() && coverageLimit == 300 && timeLimit == 3600) {
            FastestPathAlgo goToGoal = new FastestPathAlgo(exploredMap, bot, realMap);
            goToGoal.runFastestPath(RobotConstants.GOAL_ROW, RobotConstants.GOAL_COL);
        }

        FastestPathAlgo returnToStart = new FastestPathAlgo(exploredMap, bot, realMap);
        returnToStart.runFastestPath(RobotConstants.START_ROW, RobotConstants.START_COL);
    }

    /**
     * Returns true for cells that are explored and not obstacles.
     */
    private boolean isExploredNotObstacle(int r, int c) {
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell tmp = exploredMap.getCell(r, c);
            return (tmp.getIsExplored() && !tmp.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns true for cells that are explored, not virtual walls and not obstacles.
     */
    private boolean isExploredAndFree(int r, int c) {
        if (exploredMap.checkValidCoordinates(r, c)) {
            Cell b = exploredMap.getCell(r, c);
            return (b.getIsExplored() && !b.getIsVirtualWall() && !b.getIsObstacle());
        }
        return false;
    }

    /**
     * Returns the number of cells explored in the grid.
     */
    private int calculateAreaExplored() {
        int result = 0;
        for (int r = 0; r < MapConstants.MAP_ROWS; r++) {
            for (int c = 0; c < MapConstants.MAP_COLS; c++) {
                if (exploredMap.getCell(r, c).getIsExplored()) {
                    result++;
                }
            }
        }
        return result;
    }

    /**
     * Returns the number of sides possible to have image attached.
     */
    private int calculateSidesPossible() {
        int result = 0;
        for (int r = 0; r < MapConstants.MAP_ROWS; r++) {
            for (int c = 0; c < MapConstants.MAP_COLS; c++) {
                if(exploredMap.getCell(r,c).getIsExplored() && exploredMap.getCell(r,c).getIsObstacle()){
                    if (r-1>=0 && exploredMap.getCell(r-1, c).getIsExplored() && !exploredMap.getCell(r-1, c).getIsObstacle()) {
                        result++;
                    }
                    if (r+1<MapConstants.MAP_ROWS && exploredMap.getCell(r+1, c).getIsExplored() && !exploredMap.getCell(r+1, c).getIsObstacle()) {
                        result++;
                    }
                    if (c-1>=0 && exploredMap.getCell(r, c-1).getIsExplored() && !exploredMap.getCell(r, c-1).getIsObstacle()) {
                        result++;
                    }
                    if (c+1<MapConstants.MAP_COLS &&exploredMap.getCell(r, c+1).getIsExplored() && !exploredMap.getCell(r, c+1).getIsObstacle()) {
                        result++;
                    }
                }
            }
        }
        return result;
    }

    /**
     * Moves the bot, repaints the map and calls senseAndRepaint().
     */
    private void moveBot(MOVEMENT m) {
        bot.move(m);
        moves++;

        senseAndRepaint();

        if(imageProcessing && !imageMode){
            imageMode=true;
            northImage();
            eastImage();
            southImage();
            westImage();
            imageMode=false;
        }
    }

    /**
     * Sets the bot's sensors, processes the sensor data and repaints the map.
     */
    private void senseAndRepaint() {
        bot.setSensors();
        bot.sense(exploredMap, realMap);

        sendToAndroid();
        exploredMap.repaint();
    }

    /**
     * Turns the robot to the required direction.
     */
    private void turnBotDirection(DIRECTION targetDir) {
        int numOfTurn = Math.abs(bot.getRobotCurDir().ordinal() - targetDir.ordinal());
        if (numOfTurn > 2) numOfTurn = numOfTurn % 2;

        if (numOfTurn == 1) {
            if (DIRECTION.getNext(bot.getRobotCurDir()) == targetDir) {
                moveBot(MOVEMENT.RIGHT);
            } else {
                moveBot(MOVEMENT.LEFT);
            }
        } else if (numOfTurn == 2) {
            moveBot(MOVEMENT.RIGHT);
            moveBot(MOVEMENT.RIGHT);
        }
    }

    private void northImage(){
        int targetRow = bot.getRobotPosRow()+1;
        int targetCol = bot.getRobotPosCol();

        for(int i=0;i<RobotConstants.CAMERA_RANGE;i++){
            targetRow++;
            if(targetRow<0 || targetRow>=MapConstants.MAP_ROWS) return;

            Cell c = exploredMap.getCell(targetRow, targetCol);
            int index = DIRECTION.SOUTH.getIndex();

            if(c.getIsExplored() && c.getIsObstacle() && !c.getIsProcessed(index)){
                DIRECTION d = bot.getRobotCurDir();
                turnCameraDirection(DIRECTION.NORTH);

                c.setIsProcessed(index, true);
                sendToCamera(targetRow, targetCol, DIRECTION.SOUTH.toString());

                turnBotDirection(d);
                return;
            }
        }
    }

    private void eastImage(){
        int targetRow = bot.getRobotPosRow();
        int targetCol = bot.getRobotPosCol()+1;

        for(int i=0;i<RobotConstants.CAMERA_RANGE;i++){
            targetCol++;
            if(targetCol<0 || targetCol>=MapConstants.MAP_COLS) return;

            Cell c = exploredMap.getCell(targetRow, targetCol);
            int index = DIRECTION.WEST.getIndex();

            if(c.getIsExplored() && c.getIsObstacle() && !c.getIsProcessed(index)){
                DIRECTION d = bot.getRobotCurDir();
                turnCameraDirection(DIRECTION.EAST);

                c.setIsProcessed(index, true);
                sendToCamera(targetRow, targetCol, DIRECTION.WEST.toString());

                turnBotDirection(d);
                return;
            }
        }
    }

    private void southImage(){
        int targetRow = bot.getRobotPosRow()-1;
        int targetCol = bot.getRobotPosCol();

        for(int i=0;i<RobotConstants.CAMERA_RANGE;i++){
            targetRow--;
            if(targetRow<0 || targetRow>=MapConstants.MAP_ROWS) return;

            Cell c = exploredMap.getCell(targetRow, targetCol);
            int index = DIRECTION.NORTH.getIndex();

            if(c.getIsExplored() && c.getIsObstacle() && !c.getIsProcessed(index)){
                DIRECTION d = bot.getRobotCurDir();
                turnCameraDirection(DIRECTION.SOUTH);

                c.setIsProcessed(index, true);
                sendToCamera(targetRow, targetCol, DIRECTION.NORTH.toString());

                turnBotDirection(d);
                return;
            }
        }
    }

    private void westImage(){
        int targetRow = bot.getRobotPosRow();
        int targetCol = bot.getRobotPosCol()-1;

        for(int i=0;i<RobotConstants.CAMERA_RANGE;i++){
            targetCol--;
            if(targetCol<0 || targetCol>=MapConstants.MAP_COLS) return;

            Cell c = exploredMap.getCell(targetRow, targetCol);
            int index = DIRECTION.EAST.getIndex();

            if(c.getIsExplored() && c.getIsObstacle() && !c.getIsProcessed(index)){
                DIRECTION d = bot.getRobotCurDir();
                turnCameraDirection(DIRECTION.WEST);

                c.setIsProcessed(index, true);
                sendToCamera(targetRow, targetCol, DIRECTION.EAST.toString());

                turnBotDirection(d);
                return;
            }
        }
    }

    private void turnCameraDirection(DIRECTION targetDir) {
        if (RobotConstants.CAMERA_DIRECTION=="RIGHT"){
            turnBotDirection(DIRECTION.getPrevious(targetDir));
        }
        else if (RobotConstants.CAMERA_DIRECTION=="LEFT"){
            turnBotDirection(DIRECTION.getNext(targetDir));
        }
        else{
            turnBotDirection(targetDir);
        }
    }

    private void sendToAndroid(){
        String[] mapStrings = MapDescriptor.generateMapDescriptor(exploredMap);
        String robotRow = String.valueOf(bot.getRobotPosRow());
        String robotCol = String.valueOf(bot.getRobotPosCol());
        String robotDir = Character.toString(DIRECTION.print(bot.getRobotCurDir()));

        // TODO: PC -> Android
//        if(bot.getRealBot()){
//            CommMgr.getCommMgr().sendMsg(mapStrings[0] + "," + mapStrings[1] + "," + robotCol + "," + robotRow + "," + robotDir, CommMgr.AN);
//        }
    }

    private void sendToCamera(int targetRow, int targetCol, String side){
        String coordinate =  targetCol + "," + targetRow;
        takenImage++;

        if(bot.getRealBot()){
            CommMgr.getCommMgr().sendMsg(coordinate, CommMgr.IR);
            CommMgr.getCommMgr().recvMsg();
        }
        else{
            System.out.println("Take Photo: " + coordinate + " ("+ side +")");
        }
    }
}
