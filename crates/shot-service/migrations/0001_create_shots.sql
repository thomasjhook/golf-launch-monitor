-- One row per processed shot. The scalar columns exist for querying and
-- eyeballing in psql; `result` holds the full launchmonitor.v1.ShotResult
-- (including the trajectory) as protobuf and is what the API serves.
CREATE TABLE shots (
    id              BIGINT GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
    recorded_at     TIMESTAMPTZ      NOT NULL DEFAULT now(),
    shot_id         BIGINT           NOT NULL,
    ball_speed_mps  DOUBLE PRECISION NOT NULL,
    launch_deg      DOUBLE PRECISION NOT NULL,
    azimuth_deg     DOUBLE PRECISION NOT NULL,
    backspin_rpm    DOUBLE PRECISION NOT NULL,
    sidespin_rpm    DOUBLE PRECISION NOT NULL,
    carry_m         DOUBLE PRECISION NOT NULL,
    apex_m          DOUBLE PRECISION NOT NULL,
    total_m         DOUBLE PRECISION NOT NULL,
    result          BYTEA            NOT NULL
);

CREATE INDEX shots_recorded_at_idx ON shots (recorded_at DESC);
