package robot;

import map.Map;
import map.MapConstants;
import robot.RobotConstants.DIRECTION;
import robot.RobotConstants.MOVEMENT;
import utils.CommMgr;
import utils.MapDescriptor;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

// @formatter:off
/**
 * Represents the robot moving in the arena.
 *
 * The robot is represented by a 3 x 3 cell space as below:
 *
 *          ^   ^   ^
 *         SR  SR  SR
 *        [X] [X] [X]
 *   < SR [X] [X] [X] SR >
 *        [X] [X] [X]
 *
 * SR = Short Range Sensor, LR = Long Range Sensor
 */
// @formatter:on

public class Robot {
    private int posRow; // center cell
    private int posCol; // center cell
    private DIRECTION robotDir;
    private int speed;
    private final Sensor SRFrontLeft;       // north-facing front-left SR
    private final Sensor SRFrontCenter;     // north-facing front-center SR
    private final Sensor SRFrontRight;      // north-facing front-right SR
    private final Sensor SRLeft;            // west-facing left SR
    private final Sensor SRRight;           // east-facing right SR
    private boolean touchedGoal;
    private final boolean realBot;

    public Robot(int row, int col, boolean realBot) {
        posRow = row;
        posCol = col;
        robotDir = RobotConstants.START_DIR;
        speed = RobotConstants.SPEED;

        this.realBot = realBot;

        SRFrontLeft = new Sensor(RobotConstants.SENSOR_SHORT_RANGE_L, RobotConstants.SENSOR_SHORT_RANGE_H, this.posRow + 1, this.posCol - 1, this.robotDir, "SRFL");
        SRFrontCenter = new Sensor(RobotConstants.SENSOR_SHORT_RANGE_L, RobotConstants.SENSOR_SHORT_RANGE_H, this.posRow + 1, this.posCol, this.robotDir, "SRFC");
        SRFrontRight = new Sensor(RobotConstants.SENSOR_SHORT_RANGE_L, RobotConstants.SENSOR_SHORT_RANGE_H, this.posRow + 1, this.posCol + 1, this.robotDir, "SRFR");
        SRRight = new Sensor(RobotConstants.SENSOR_SHORT_RANGE_L, RobotConstants.SENSOR_SHORT_RANGE_H, this.posRow + 1, this.posCol + 1, findNewDirection(MOVEMENT.RIGHT), "SRR");
        SRLeft = new Sensor(RobotConstants.SENSOR_SHORT_RANGE_L, RobotConstants.SENSOR_SHORT_RANGE_H, this.posRow, this.posCol - 1, findNewDirection(MOVEMENT.LEFT), "SRL");
    }

    public void setRobotPos(int row, int col) {
        posRow = row;
        posCol = col;
    }

    public int getRobotPosRow() {
        return posRow;
    }

    public int getRobotPosCol() {
        return posCol;
    }

    public void setRobotDir(DIRECTION dir) {
        robotDir = dir;
    }

    public void setSpeed(int speed) {
        this.speed = speed;
    }

    public DIRECTION getRobotCurDir() {
        return robotDir;
    }

    public boolean getRealBot() {
        return realBot;
    }

    private void updateTouchedGoal() {
        if (this.getRobotPosRow() == MapConstants.GOAL_ROW && this.getRobotPosCol() == MapConstants.GOAL_COL)
            this.touchedGoal = true;
    }

    public boolean getTouchedGoal() {
        return this.touchedGoal;
    }

    /**
     * Takes in a MOVEMENT and moves the robot accordingly by changing its position and direction. Sends the movement
     * if this.realBot is set.
     */
    public void move(MOVEMENT m, boolean sendMoveToAndroid) {
        //System.out.println("move (Robot) activated");
        if (!realBot) {
            // Emulate real movement by pausing execution.
            try {
                TimeUnit.MILLISECONDS.sleep(speed);
            } catch (InterruptedException e) {
                System.out.println("Something went wrong in Robot.move()!");
            }
        }

        switch (m) {
            case FORWARD:
                switch (robotDir) {
                    case NORTH:
                        posRow++;
                        break;
                    case EAST:
                        posCol++;
                        break;
                    case SOUTH:
                        posRow--;
                        break;
                    case WEST:
                        posCol--;
                        break;
                }
                break;
            case BACKWARD:
                switch (robotDir) {
                    case NORTH:
                        posRow--;
                        break;
                    case EAST:
                        posCol--;
                        break;
                    case SOUTH:
                        posRow++;
                        break;
                    case WEST:
                        posCol++;
                        break;
                }
                break;
            case RIGHT:
            case LEFT:
                robotDir = findNewDirection(m);
                break;
            case FORCEDLEFT:
                switch (robotDir) {
                    case NORTH:
                        posCol--;
                        break;
                    case EAST:
                        posRow++;
                        break;
                    case SOUTH:
                        posCol++;
                        break;
                    case WEST:
                        posRow--;
                        break;
                }
                break;
            case FORCEDRIGHT:
                switch (robotDir) {
                    case NORTH:
                        posCol++;
                        break;
                    case EAST:
                        posRow--;
                        break;
                    case SOUTH:
                        posCol--;
                        break;
                    case WEST:
                        posRow++;
                        break;
                }
                break;
            case CALIBRATE:
                break;
            default:
                System.out.println("Error in Robot.move()!");
                break;
        }

        // System.out.println(Character.toString(MOVEMENT.print(m)));

        if (realBot)
            sendMovement(m, sendMoveToAndroid);
        // else System.out.println("Move: " + MOVEMENT.print(m));

        updateTouchedGoal();
    }

    /**
     * Overloaded method that calls this.move(MOVEMENT m, boolean sendMoveToAndroid = true).
     */
    public void move(MOVEMENT m) {
        //System.out.println("move (shorter version) Activated");
        this.move(m, true);
    }

    /**
     * Sends a number instead of 'F' for multiple continuous forward movements.
     */
    public void moveForwardMultiple(int count) {
        //System.out.println("moveForwardMultiple Activated");
        if (count == 1) {
            move(MOVEMENT.FORWARD);
        } else {
            CommMgr comm = CommMgr.getCommMgr();
            if (count == 10) {
                comm.sendMsg("0", CommMgr.AR);
            } else if (count < 10) {
                comm.sendMsg(Integer.toString(count), CommMgr.AR);
            }

            switch (robotDir) {
                case NORTH:
                    posRow += count;
                    break;
                case EAST:
                    posCol += count;
                    break;
                case SOUTH:
                    posRow += count;
                    break;
                case WEST:
                    posCol += count;
                    break;
            }

            // Sends robot position to Android
            /** String robotRow = String.valueOf(this.getRobotPosRow());
            String robotCol = String.valueOf(this.getRobotPosCol());
            String robotDir = Character.toString(DIRECTION.print(this.getRobotCurDir()));
            comm.sendMsg(robotRow + "" + robotCol + "" + robotDir, CommMgr.AN); **/
        }
    }

    /**
     * Uses the CommMgr to send the next movement to the robot.
     */
    private void sendMovement(MOVEMENT m, boolean sendMoveToAndroid) {
        //System.out.println("sendMovement Activated");
        CommMgr comm = CommMgr.getCommMgr();
        comm.sendMsg(Character.toString(MOVEMENT.print(m)), CommMgr.AR);

        // Sends position of Robot to Android
        /**if (m != MOVEMENT.CALIBRATE && sendMoveToAndroid) {
            String robotRow = String.valueOf(this.getRobotPosRow());
            String robotCol = String.valueOf(this.getRobotPosCol());
            String robotDir = Character.toString(DIRECTION.print(this.getRobotCurDir()));
            comm.sendMsg(robotRow + "" + robotCol + "" + robotDir, CommMgr.AN);
        }**/
    }

    /**
     * Sets the sensors' position and direction values according to the robot's current position and direction.
     */
    public void setSensors() {
        //System.out.println("setSensors Activated");
        switch (robotDir) {
            case NORTH:
                SRFrontLeft.setSensor(this.posRow + 1, this.posCol - 1, this.robotDir);
                SRFrontCenter.setSensor(this.posRow + 1, this.posCol, this.robotDir);
                SRFrontRight.setSensor(this.posRow + 1, this.posCol + 1, this.robotDir);
                SRLeft.setSensor(this.posRow, this.posCol - 1, findNewDirection(MOVEMENT.LEFT));
                SRRight.setSensor(this.posRow, this.posCol + 1, findNewDirection(MOVEMENT.RIGHT));
                break;
            case EAST:
                SRFrontLeft.setSensor(this.posRow + 1, this.posCol + 1, this.robotDir);
                SRFrontCenter.setSensor(this.posRow, this.posCol + 1, this.robotDir);
                SRFrontRight.setSensor(this.posRow - 1, this.posCol + 1, this.robotDir);
                SRLeft.setSensor(this.posRow + 1, this.posCol, findNewDirection(MOVEMENT.LEFT));
                SRRight.setSensor(this.posRow - 1, this.posCol, findNewDirection(MOVEMENT.RIGHT));
                break;
            case SOUTH:
                SRFrontLeft.setSensor(this.posRow - 1, this.posCol + 1, this.robotDir);
                SRFrontCenter.setSensor(this.posRow - 1, this.posCol, this.robotDir);
                SRFrontRight.setSensor(this.posRow - 1, this.posCol - 1, this.robotDir);
                SRLeft.setSensor(this.posRow, this.posCol + 1, findNewDirection(MOVEMENT.LEFT));
                SRRight.setSensor(this.posRow, this.posCol - 1, findNewDirection(MOVEMENT.RIGHT));
                break;
            case WEST:
                SRFrontLeft.setSensor(this.posRow - 1, this.posCol - 1, this.robotDir);
                SRFrontCenter.setSensor(this.posRow, this.posCol - 1, this.robotDir);
                SRFrontRight.setSensor(this.posRow + 1, this.posCol - 1, this.robotDir);
                SRLeft.setSensor(this.posRow - 1, this.posCol, findNewDirection(MOVEMENT.LEFT));
                SRRight.setSensor(this.posRow + 1, this.posCol, findNewDirection(MOVEMENT.RIGHT));
                break;
        }
    }

    /**
     * Uses the current direction of the robot and the given movement to find the new direction of the robot.
     */
    private DIRECTION findNewDirection(MOVEMENT m) {
        //System.out.println("findNewDirection Activated");
        if (m == MOVEMENT.RIGHT) {
            return DIRECTION.getNext(robotDir);
        } else {
            return DIRECTION.getPrevious(robotDir);
        }
    }

    /**
     * Calls the .sense() method of all the attached sensors and stores the received values in an integer array.
     *
     * @return [SRLeft, SRFrontLeft, SRFrontCenter, SRFrontRight, SRRight]
     */
    public int[] sense(Map explorationMap, Map realMap) {
        //System.out.println("sense Activated");
        int[] result = new int[5];

        if (!realBot) {
            result[0] = SRLeft.sense(explorationMap, realMap);
            result[1] = SRFrontLeft.sense(explorationMap, realMap);
            result[2] = SRFrontCenter.sense(explorationMap, realMap);
            result[3] = SRFrontRight.sense(explorationMap, realMap);
            result[4] = SRRight.sense(explorationMap, realMap);
        } else {
            CommMgr comm = CommMgr.getCommMgr();
            String msg = comm.recvMsg();    // Input in the form of xx, xx, xx, xx, xx, xx
            String[] msgArr = msg.split(",");   // Splits the incoming message into an array based on the position of ','

            System.out.println("Received a message here");

            // Convert the values in the incoming message from strings to double
            /*double[] msgArr2 = new double[6];
            for (int i=0; i<5; i++){
                msgArr2[i] = Double.parseDouble(msgArr[i]);
            }

            // Process the double values to the values the algorithm will use
            int[] msgArr3 = new int[6];
            for (int i=0; i<5; i++){
                if (msgArr2[i] > 41){
                    msgArr3[i] = -1;
                }
                else {
                    msgArr3[i] = (int) (msgArr2[i]/10);
                }
            }*/

            /*if (msgArr[0].equals(CommMgr.AR)) {
                result[0] = msgArr3[0];
                result[1] = msgArr3[1];
                result[2] = msgArr3[2];
                result[3] = msgArr3[3];
                result[4] = msgArr3[4];
            }*/

            for (int i=0; i<5; i++){
                result[i] = Integer.parseInt(msgArr[i]);
            }

            SRLeft.senseReal(explorationMap, result[0]);
            SRFrontLeft.senseReal(explorationMap, result[1]);
            SRFrontCenter.senseReal(explorationMap, result[2]);
            SRFrontRight.senseReal(explorationMap, result[3]);
            SRRight.senseReal(explorationMap, result[4]);

            String[] mapStrings = MapDescriptor.generateMapDescriptor(explorationMap);
            String robotRow = String.valueOf(this.getRobotPosRow());
            String robotCol = String.valueOf(this.getRobotPosCol());
            String robotDir = Character.toString(DIRECTION.print(this.getRobotCurDir()));
            comm.sendMsg(mapStrings[0] + "," + mapStrings[1] + "," + robotRow + "," + robotCol + "," + robotDir, CommMgr.AN);
        }

        System.out.println(Arrays.toString(result));

        return result;
    }
}
