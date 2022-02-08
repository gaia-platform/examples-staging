
# Dummy data for testing display.

motion = {
  "distance": 5.0,
  "turn_angle": 1.0
}

position = {
  "x": 280.0,
  "y": 280.0,
  "heading": 315.0
}

world_map = {
  "description": "box with door and table",
  "width": 1000,
  "height": 600,
  "world": [
    {
      "name": "Outer wall",
      "vertices": [
        { "x":    1.0, "y":   1.0 },
        { "x":  450.0, "y":   1.0 },
        { "x":  450.0, "y":  10.0 },
        { "x":   10.0, "y":  10.0 },
        { "x":   10.0, "y": 590.0 },
        { "x":  990.0, "y": 590.0 },
        { "x":  990.0, "y":  10.0 },
        { "x":  550.0, "y":  10.0 },
        { "x":  550.0, "y":   1.0 },
        { "x":  999.0, "y":   1.0 },
        { "x":  999.0, "y": 599.0 },
        { "x":    1.0, "y": 599.0 },
        { "x":    1.0, "y":   1.0 }
      ]
    },
    {
      "name": "Table",
      "vertices": [
        { "x":  125.0, "y": 125.0 },
        { "x":  200.0, "y": 125.0 },
        { "x":  200.0, "y": 200.0 },
        { "x":  125.0, "y": 200.0 },
        { "x":  125.0, "y": 125.0 }
      ]
    }
  ]
}

sensor_data = {
  "position": {
    "x": 280,
    "y": 280,
    "heading_deg": 315
  },
  "motion": {
    "dx": 0,
    "dy": 0,
    "dhead_deg": 0
  },
  "senses": {
    "est_position": {
      "x": 285,
      "y": 280,
      "heading_deg": 325
    },
    "est_motion": {
      "dx": 0,
      "dy": 0,
      "dhead_deg": 0
    },
    "lidar": [
      { "bearing": -30, "range": 250 },
      { "bearing": -28, "range": 260 },
      { "bearing": -26, "range": 248 },
      { "bearing": -24, "range": 255 },
      { "bearing": -22, "range": 142 },
      { "bearing": -20, "range": 124 },
      { "bearing": -18, "range": 120 },
      { "bearing": -16, "range": 102 },
      { "bearing": -14, "range": 104 },
      { "bearing": -12, "range":  95 },
      { "bearing": -10, "range":  91 },
      { "bearing":  -8, "range":  87 },
      { "bearing":  -6, "range":  83 },
      { "bearing":  -4, "range":  81 },
      { "bearing":  -2, "range":  79 },
      { "bearing":   0, "range":  73 },
      { "bearing":   2, "range":  75 },
      { "bearing":   4, "range":  84 },
      { "bearing":   6, "range":  84 },
      { "bearing":   8, "range":  86 },
      { "bearing":  10, "range":  93 },
      { "bearing":  12, "range":  98 },
      { "bearing":  14, "range":  99 },
      { "bearing":  16, "range": 107 },
      { "bearing":  18, "range": 123 },
      { "bearing":  20, "range": 133 },
      { "bearing":  22, "range": 139 },
      { "bearing":  24, "range": 262 },
      { "bearing":  26, "range": 250 },
      { "bearing":  28, "range": 248 },
      { "bearing":  30, "range": 258 }
    ]
  }
}

#      { "bearing":   0, "range":  74 },
#      { "bearing":   2, "range":  76 },
#      { "bearing":   4, "range":  80 },
#      { "bearing":   6, "range":  84 },
#      { "bearing":   8, "range":  88 },
#      { "bearing":  10, "range":  90 },
#      { "bearing":  12, "range":  96 },
#      { "bearing":  14, "range": 102 },
#      { "bearing":  16, "range": 110 },
#      { "bearing":  18, "range": 118 },
#      { "bearing":  20, "range": 128 },
#      { "bearing":  22, "range": 140 },
#      { "bearing":  24, "range": 254 },
#      { "bearing":  26, "range": 253 },
#      { "bearing":  28, "range": 242 },
#      { "bearing":  30, "range": 251 }
