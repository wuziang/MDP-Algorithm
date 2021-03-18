package robot;

/**
 * Constants used in this package.
 */

public class RobotConstants {
    public static final int GOAL_ROW = 18;                          // row no. of goal cell
    public static final int GOAL_COL = 13;                          // col no. of goal cell

    public static final int START_ROW = 1;                          // row no. of start cell
    public static final int START_COL = 1;                          // col no. of start cell

    public static final int MOVE_COST = 10;                         // cost of FORWARD, BACKWARD movement
    public static final int TURN_COST = 20;                         // cost of RIGHT, LEFT movement
    public static final int SPEED = 50;                            // delay between movements (ms)

    public static final DIRECTION START_DIR = DIRECTION.NORTH;      // start direction

    public static final int SENSOR_SHORT_RANGE_L = 1;               // lower range of short range sensor (cells)
    public static final int SENSOR_SHORT_RANGE_H = 3;               // upper range of short range sensor (cells)

    public static final int CAMERA_RANGE = 2;                       // upper range of camera (cells)
    public static final String CAMERA_DIRECTION = "RIGHT";          // direction of camera (cells)

    public static final int CALIBRATE_THRESHOLD = 5;
    public static final int INFINITE_COST = 9999;

    public enum DIRECTION {
        NORTH(0), EAST(1), SOUTH(2), WEST(3);

        int index;
        DIRECTION(int index){
            this.index = index;
        }

        public int getIndex() {
            return index;
        }

        public static DIRECTION getNext(DIRECTION curDirection) {
            return values()[(curDirection.ordinal() + 1) % values().length];
        }

        public static DIRECTION getPrevious(DIRECTION curDirection) {
            return values()[(curDirection.ordinal() + values().length - 1) % values().length];
        }

        public static char print(DIRECTION d) {
            switch (d) {
                case NORTH:
                    return 'N';
                case EAST:
                    return 'E';
                case SOUTH:
                    return 'S';
                case WEST:
                    return 'W';
                default:
                    return 'X';
            }
        }
    }

    public enum MOVEMENT {
        FORWARD, BACKWARD, RIGHT, LEFT, CALIBRATE, FORCEDLEFT, FORCEDRIGHT, ERROR;

        public static char print(MOVEMENT m) {
            switch (m) {
                case FORWARD:
                    return 'F';
                case BACKWARD:
                    return 'B';
                case RIGHT:
                    return 'R';
                case LEFT:
                    return 'L';
                case FORCEDLEFT:
                    return 'A';
                case FORCEDRIGHT:
                    return 'D';
                case CALIBRATE:
                    return 'C';
                case ERROR:
                default:
                    return 'E';
            }
        }
    }
}
