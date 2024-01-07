package org.bitbuckets;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * WRT blue as 0,0
 * https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024FieldDrawings.pdf
 */
public class FieldConstants {

    static final double X_FIELD_METERS = 16.541;
    static final double Y_FIELD_METERS = 8.211;

    static final double Y_MIDPOINT_TO_STEREO_METERS = 1.45;
    static final double X_WALL_TO_AMP_METERS = 1.93;

    static final double STAGE_TRIANGLE_BASE_METERS = 3.115;
    static final double STAGE_TRIANGLE_HEIGHT_METERS = 2.697;
    //Distance to red and blue alliance walls are different
    static final double BLUE_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS = 3.073 + STAGE_TRIANGLE_HEIGHT_METERS * 2.0 / 3.0;
    static final double RED_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS = 3.061 + STAGE_TRIANGLE_HEIGHT_METERS * 2.0 / 3.0;

    static final double ALLIANCE_WALL_TO_WING_NOTES_METERS = 2.896;
    static final double WING_NOTES_SEPERATION_METERS = 1.448;
    static final double CENTER_NOTES_SEPERATION_METERS = 1.676;

    static final double X_SOURCE_METERS = 1.832;
    static final double Y_SOURCE_METERS = 1.075;
    static final double SOURCE_WALL_METERS = 2.026; //length of the entire wall(inside hinges)

    static final Translation2d BOTTOM_LEFT = new Translation2d(0,0);
    static final Translation2d TOP_LEFT = new Translation2d(0, Y_FIELD_METERS);

    static final Translation2d BLUE_STEREO = new Translation2d(0, (Y_FIELD_METERS / 2d) + Y_MIDPOINT_TO_STEREO_METERS);
    static final Translation2d RED_STEREO = new Translation2d(X_FIELD_METERS, (Y_FIELD_METERS / 2d) + Y_MIDPOINT_TO_STEREO_METERS);

    static final Translation2d BLUE_AMP = new Translation2d(X_WALL_TO_AMP_METERS, Y_FIELD_METERS);
    static final Translation2d RED_AMP = new Translation2d(X_FIELD_METERS - X_WALL_TO_AMP_METERS, Y_FIELD_METERS);

    Translation2d BLUE_STAGE_CENTER = new Translation2d(BLUE_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS, (Y_FIELD_METERS / 2d));
    Translation2d RED_STAGE_CENTER = new Translation2d((X_FIELD_METERS - RED_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS), (Y_FIELD_METERS / 2d));

    Translation2d BLUE_STAGE_LEFT = new Translation2d((BLUE_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS - STAGE_TRIANGLE_HEIGHT_METERS * 2/3), (Y_FIELD_METERS / 2d));
    Translation2d BLUE_STAGE_TOP = new Translation2d((BLUE_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS + STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (Y_FIELD_METERS / 2d + STAGE_TRIANGLE_BASE_METERS / 2d));
    Translation2d BLUE_STAGE_BOTTOM = new Translation2d((BLUE_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS + STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (Y_FIELD_METERS / 2d - STAGE_TRIANGLE_BASE_METERS / 2d));

    Translation2d RED_STAGE_RIGHT = new Translation2d((X_FIELD_METERS - RED_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS + STAGE_TRIANGLE_HEIGHT_METERS * 2/3), (Y_FIELD_METERS / 2d));
    Translation2d RED_STAGE_TOP = new Translation2d((X_FIELD_METERS - RED_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS - STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (Y_FIELD_METERS / 2d + STAGE_TRIANGLE_BASE_METERS / 2d));
    Translation2d RED_STAGE_BOTTOM = new Translation2d((X_FIELD_METERS - RED_ALLIANCE_WALL_TO_STAGE_CENTROID_METERS - STAGE_TRIANGLE_HEIGHT_METERS * 1/3), (Y_FIELD_METERS / 2d - STAGE_TRIANGLE_BASE_METERS / 2d));

    Translation2d BLUE_WING_NOTE_BOTTOM = new Translation2d(ALLIANCE_WALL_TO_WING_NOTES_METERS, Y_FIELD_METERS / 2d);
    Translation2d BLUE_WING_NOTE_MID = new Translation2d(ALLIANCE_WALL_TO_WING_NOTES_METERS,    Y_FIELD_METERS / 2d + WING_NOTES_SEPERATION_METERS);
    Translation2d BLUE_WING_NOTE_TOP = new Translation2d(ALLIANCE_WALL_TO_WING_NOTES_METERS, Y_FIELD_METERS / 2d + WING_NOTES_SEPERATION_METERS * 2);

    Translation2d RED_WING_NOTE_BOTTOM = new Translation2d(X_FIELD_METERS - ALLIANCE_WALL_TO_WING_NOTES_METERS, Y_FIELD_METERS / 2d);
    Translation2d RED_WING_NOTE_MID = new Translation2d(X_FIELD_METERS - ALLIANCE_WALL_TO_WING_NOTES_METERS, Y_FIELD_METERS / 2d + WING_NOTES_SEPERATION_METERS);
    Translation2d RED_WING_NOTE_TOP = new Translation2d(X_FIELD_METERS - ALLIANCE_WALL_TO_WING_NOTES_METERS, Y_FIELD_METERS / 2d + WING_NOTES_SEPERATION_METERS * 2);

    Translation2d CENTER_NOTE_BOTTOM = new Translation2d(X_FIELD_METERS / 2d, Y_FIELD_METERS / 2d - CENTER_NOTES_SEPERATION_METERS * 2);
    Translation2d CENTER_NOTE_MID_BOTTOM = new Translation2d(X_FIELD_METERS / 2d, Y_FIELD_METERS / 2d - CENTER_NOTES_SEPERATION_METERS);
    Translation2d CENTER_NOTE_MID = new Translation2d(X_FIELD_METERS / 2d, Y_FIELD_METERS / 2d);
    Translation2d CENTER_NOTE_MID_TOP = new Translation2d(X_FIELD_METERS / 2d, Y_FIELD_METERS / 2d + CENTER_NOTES_SEPERATION_METERS);
    Translation2d CENTER_NOTE_TOP = new Translation2d(X_FIELD_METERS / 2d, Y_FIELD_METERS / 2d + CENTER_NOTES_SEPERATION_METERS * 2);

    Translation2d RED_SOURCE_CENTER = new Translation2d((X_SOURCE_METERS / 2d), (Y_SOURCE_METERS / 2d));
    Translation2d BLUE_SOURCE_CENTER = new Translation2d((X_FIELD_METERS - X_SOURCE_METERS / 2d), (Y_SOURCE_METERS / 2d));
}
