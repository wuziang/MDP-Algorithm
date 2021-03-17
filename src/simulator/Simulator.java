package simulator;

import algorithms.ExplorationAlgo;
import algorithms.FastestPathAlgo;
import map.Map;
import map.MapConstants;
import robot.Robot;
import robot.RobotConstants;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;

import static utils.MapDescriptor.*;

/**
 * Simulator for robot navigation in virtual arena.
 */

public class Simulator {
    private static JFrame _appFrame = null;         // application JFrame

    private static JPanel _mapCards = null;         // JPanel for map views
    private static JPanel _buttons = null;          // JPanel for buttons

    private static Robot bot;

    private static Map realMap = null;              // real map
    private static Map exploredMap = null;          // exploration map

    private static boolean loadedMap = false;
    private static String filename = "Sample5";

    private static int waypointRow = 7;
    private static int waypointCol = 8;

    private static int timeLimit = 3600;            // time limit
    private static int coverageLimit = 300;         // coverage limit
    private static boolean pledgeEnabled = true;

    /**
     * Initialises the different maps and displays the application.
     */
    public static void main(String[] args) {

        bot = new Robot(RobotConstants.START_ROW, RobotConstants.START_COL, false);

        realMap = new Map(bot);
        realMap.setAllUnexplored();

        exploredMap = new Map(bot);
        exploredMap.setAllUnexplored();

        displayEverything();
    }

    /**
     * Initialises the different parts of the application.
     */
    private static void displayEverything() {
        // Initialise main frame for display
        _appFrame = new JFrame();
        _appFrame.setTitle("Simulator");
        _appFrame.setSize(new Dimension(700, 750));
        _appFrame.setResizable(false);

        // Center the main frame in the middle of the screen
        Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
        _appFrame.setLocation(dim.width / 2 - _appFrame.getSize().width / 2, dim.height / 2 - _appFrame.getSize().height / 2);

        // Create the CardLayout for storing the different maps
        _mapCards = new JPanel(new CardLayout());

        // Create the JPanel for the buttons
        _buttons = new JPanel();

        // Add _mapCards & _buttons to the main frame's content pane
        Container contentPane = _appFrame.getContentPane();
        contentPane.add(_mapCards, BorderLayout.CENTER);
        contentPane.add(_buttons, BorderLayout.PAGE_END);

        // Initialize the main map view
        initMainLayout();

        // Initialize the buttons
        initButtonsLayout();

        // Display the application
        _appFrame.setVisible(true);
        _appFrame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
    }

    /**
     * Initialises the main map view by adding the different maps as cards in the CardLayout. Displays realMap
     * by default.
     */
    private static void initMainLayout() {
        _mapCards.add(realMap, "REAL_MAP");
        _mapCards.add(exploredMap, "EXPLORATION");

        CardLayout cl = ((CardLayout) _mapCards.getLayout());
        cl.show(_mapCards, "REAL_MAP");
    }

    /**
     * Initialises the JPanel for the buttons.
     */
    private static void initButtonsLayout() {
        _buttons.setLayout(new GridLayout(2,1));
        addButtons();
    }

    private static void loadDefaultMap(){
        if(!loadedMap) {
            loadMap(realMap, filename);
            realMap.setWaypoint(waypointRow, waypointCol);

            realMap.repaint();
        }
    }

    /**
     * Helper method to set particular properties for all the JButtons.
     */
    private static void formatButton(JButton btn) {
        btn.setFont(new Font("Arial", Font.BOLD, 13));
        btn.setFocusPainted(false);
    }

    /**
     * Initialises and adds the five main buttons. Also creates the relevant classes (for multithreading) and JDialogs
     * (for user input) for the different functions of the buttons.
     */
    private static void addButtons() {
        JButton btn_LoadMap = new JButton("Load Map");
        formatButton(btn_LoadMap);

        btn_LoadMap.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadedMap = true;

                JDialog loadMapDialog = new JDialog(_appFrame, "Load Map", true);
                loadMapDialog.setSize(600, 80);
                loadMapDialog.setLayout(new FlowLayout());

                final JTextField loadTF = new JTextField(15);
                final JTextField loadWaypoint = new JTextField(10);

                JButton loadMapButton = new JButton("Load");

                loadMapButton.addMouseListener(new MouseAdapter() {
                    public void mousePressed(MouseEvent e) {
                        loadMapDialog.setVisible(false);
                        loadMap(realMap, loadTF.getText());

                        String waypoint=loadWaypoint.getText();
                        waypointRow =Integer.parseInt(waypoint.substring(0, waypoint.indexOf(',')));
                        waypointCol =Integer.parseInt(waypoint.substring(waypoint.indexOf(',')+1));
                        realMap.setWaypoint(waypointRow, waypointCol);

                        CardLayout cl = ((CardLayout) _mapCards.getLayout());
                        cl.show(_mapCards, "REAL_MAP");
                        realMap.repaint();
                    }
                });

                loadMapDialog.add(new JLabel("File: "));
                loadMapDialog.add(loadTF);

                loadMapDialog.add(new JLabel("Waypoint: "));
                loadMapDialog.add(loadWaypoint);
                loadMapDialog.add(loadMapButton);

                loadMapDialog.setVisible(true);
            }
        });

        _buttons.add(btn_LoadMap);

        // FastestPath Class for Multithreading
        class FastestPath extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                bot.setRobotPos(RobotConstants.START_ROW, RobotConstants.START_COL);
                bot.setRobotDir(RobotConstants.DIRECTION.NORTH);

                realMap.repaint();

                FastestPathAlgo fastestPathToWayPoint;
                fastestPathToWayPoint = new FastestPathAlgo(realMap, bot);
                fastestPathToWayPoint.runFastestPath(waypointRow, waypointCol);

                bot.setRobotPos(waypointRow, waypointCol);
                realMap.repaint();

                FastestPathAlgo fastestPathToGoal;
                fastestPathToGoal = new FastestPathAlgo(realMap, bot);
                fastestPathToGoal.runFastestPath(RobotConstants.GOAL_ROW, RobotConstants.GOAL_COL);

                return 111;
            }
        }

        // Exploration Class for Multithreading
        class Exploration extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                int row, col;

                row = RobotConstants.START_ROW;
                col = RobotConstants.START_COL;

                bot.setRobotPos(row, col);
                exploredMap.repaint();

                ExplorationAlgo exploration;
                exploration = new ExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);

                exploration.setPledgeEnabled(pledgeEnabled);

                exploration.runExploration();
                generateMapDescriptor(exploredMap);

                return 222;
            }
        }

        // Image Exploration Class for Multithreading
        class ImageExploration extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                int row, col;

                row = RobotConstants.START_ROW;
                col = RobotConstants.START_COL;

                bot.setRobotPos(row, col);
                exploredMap.repaint();

                ExplorationAlgo image;
                image = new ExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);

                image.setImageProcessing(true);
                image.setPledgeEnabled(pledgeEnabled);

                image.runExploration();
                generateMapDescriptor(exploredMap);

                return 333;
            }
        }

        // Fastest Path Button
        JButton btn_FastestPath = new JButton("Fastest Path");
        formatButton(btn_FastestPath);
        btn_FastestPath.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadDefaultMap();

                CardLayout cl = ((CardLayout) _mapCards.getLayout());
                cl.show(_mapCards, "REAL_MAP");
                new FastestPath().execute();
            }
        });
        _buttons.add(btn_FastestPath);

        // Exploration Button
        JButton btn_Exploration = new JButton("Exploration");
        formatButton(btn_Exploration);
        btn_Exploration.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadDefaultMap();

                CardLayout cl = ((CardLayout) _mapCards.getLayout());
                cl.show(_mapCards, "EXPLORATION");
                new Exploration().execute();
            }
        });
        _buttons.add(btn_Exploration);

        // Image Exploration Button
        JButton btn_Image_Exploration = new JButton("Image Processing");
        formatButton(btn_Image_Exploration);
        btn_Image_Exploration.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadDefaultMap();

                CardLayout cl = ((CardLayout) _mapCards.getLayout());
                cl.show(_mapCards, "EXPLORATION");
                new ImageExploration().execute();
            }
        });
        _buttons.add(btn_Image_Exploration);

        // TimeExploration Class for Multithreading
        class TimeExploration extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                bot.setRobotPos(RobotConstants.START_ROW, RobotConstants.START_COL);
                exploredMap.repaint();

                ExplorationAlgo timeExplo = new ExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);
                timeExplo.runExploration();

                generateMapDescriptor(exploredMap);
                return 444;
            }
        }

        // Time-limited Exploration Button
        JButton btn_TimeExploration = new JButton("Time-Limited");
        formatButton(btn_TimeExploration);
        btn_TimeExploration.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadDefaultMap();

                JDialog timeExploDialog = new JDialog(_appFrame, "Time-Limited Exploration", true);
                timeExploDialog.setSize(400, 60);
                timeExploDialog.setLayout(new FlowLayout());

                final JTextField timeTF = new JTextField(5);
                JButton timeSaveButton = new JButton("Run");

                timeSaveButton.addMouseListener(new MouseAdapter() {
                    public void mousePressed(MouseEvent e) {
                        timeExploDialog.setVisible(false);
                        String time = timeTF.getText();
                        String[] timeArr = time.split(":");
                        timeLimit = (Integer.parseInt(timeArr[0]) * 60) + Integer.parseInt(timeArr[1]);
                        CardLayout cl = ((CardLayout) _mapCards.getLayout());
                        cl.show(_mapCards, "EXPLORATION");
                        new TimeExploration().execute();
                    }
                });

                timeExploDialog.add(new JLabel("Time Limit (MM:SS): "));
                timeExploDialog.add(timeTF);
                timeExploDialog.add(timeSaveButton);
                timeExploDialog.setVisible(true);
            }
        });
        _buttons.add(btn_TimeExploration);

        // CoverageExploration Class for Multithreading
        class CoverageExploration extends SwingWorker<Integer, String> {
            protected Integer doInBackground() throws Exception {
                bot.setRobotPos(RobotConstants.START_ROW, RobotConstants.START_COL);
                exploredMap.repaint();

                ExplorationAlgo coverageExplo = new ExplorationAlgo(exploredMap, realMap, bot, coverageLimit, timeLimit);
                coverageExplo.runExploration();

                generateMapDescriptor(exploredMap);
                return 555;
            }
        }

        // Coverage-limited Exploration Button
        JButton btn_CoverageExploration = new JButton("Coverage-Limited");
        formatButton(btn_CoverageExploration);
        btn_CoverageExploration.addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                loadDefaultMap();

                JDialog coverageExploDialog = new JDialog(_appFrame, "Coverage-Limited Exploration", true);
                coverageExploDialog.setSize(400, 60);
                coverageExploDialog.setLayout(new FlowLayout());

                final JTextField coverageTF = new JTextField(5);
                JButton coverageSaveButton = new JButton("Run");

                coverageSaveButton.addMouseListener(new MouseAdapter() {
                    public void mousePressed(MouseEvent e) {
                        coverageExploDialog.setVisible(false);
                        coverageLimit = (int) ((Integer.parseInt(coverageTF.getText())) * MapConstants.MAP_SIZE / 100.0);
                        new CoverageExploration().execute();
                        CardLayout cl = ((CardLayout) _mapCards.getLayout());
                        cl.show(_mapCards, "EXPLORATION");
                    }
                });

                coverageExploDialog.add(new JLabel("Coverage Limit (%): "));
                coverageExploDialog.add(coverageTF);
                coverageExploDialog.add(coverageSaveButton);
                coverageExploDialog.setVisible(true);
            }
        });

        _buttons.add(btn_CoverageExploration);
    }
}