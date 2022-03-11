database gaia_bot

table ego (
    id uint8 unique,
    name string,

    now uint64,

    range_sensors
        references range_sensors[],
    face_tracks
        references face_tracks[]
)

table neck_pose (
    rotation float,
    lift float
)

table range_sensors (
    id uint8,
    name string,

    timestamp uint64,
    distance uint16,

    ego
        references ego
            where range_sensors.ego_id = ego.id,
    ego_id uint8
)

table face_tracks (
    id uint32 unique,
    name string,

    start_timestamp uint64,
    last_timestamp uint64,

    dx float,
    dy float,
    dz float,

    face_detections references face_detections[],
    detection_count uint16,

    remove_flag bool,

    ego
        references ego
            where face_tracks.ego_id = ego.id,
    ego_id uint8
)

table face_detections (
    timestamp uint64,

    -- location of face relative to current field of view of camera
    -- (0,0,0) is center, (-1,-1,0) is upper left, (1,1,0) is lower right
    -- dz is currently always 0
    dx float,
    dy float,
    dz float,

    width float,
    height float,
    depth float,

    remove_flag bool,

    face_track
        references face_tracks
            where face_detections.face_track_id = face_tracks.id,
    face_track_id uint32
)
;
