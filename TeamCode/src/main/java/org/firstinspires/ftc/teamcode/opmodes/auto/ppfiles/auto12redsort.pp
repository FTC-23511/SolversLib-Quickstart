{
  "startPoint": {
    "x": 104.2,
    "y": 135.7,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-3za6r8pg3j5",
      "name": "Path 1",
      "endPoint": {
        "x": 88.4,
        "y": 81,
        "heading": "linear",
        "startDeg": 90,
        "endDeg": 49,
        "degrees": 49
      },
      "controlPoints": [],
      "color": "#A6DC59",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk2y7kl-a47zi3",
      "endPoint": {
        "x": 125,
        "y": 83,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#D6968A",
      "name": "Path 12",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkk2yfk3-v965h5",
      "endPoint": {
        "x": 88.4,
        "y": 81,
        "heading": "tangential",
        "reverse": true
      },
      "controlPoints": [],
      "color": "#BDAA75",
      "name": "Path 13",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkjcnzkz-jjjotj",
      "name": "Path 2",
      "endPoint": {
        "x": 136,
        "y": 58,
        "heading": "linear",
        "reverse": false,
        "startDeg": 49,
        "endDeg": 49
      },
      "controlPoints": [
        {
          "x": 90.52286874154262,
          "y": 33.65480378890391
        },
        {
          "x": 105.86576454668469,
          "y": 66.51082543978347
        }
      ],
      "color": "#BDC655",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkjcp9wl-czdcgg",
      "name": "Path 3",
      "endPoint": {
        "x": 88.4,
        "y": 81,
        "heading": "linear",
        "reverse": false,
        "startDeg": 49,
        "endDeg": 50
      },
      "controlPoints": [
        {
          "x": 89,
          "y": 58
        },
        {
          "x": 133.5,
          "y": 61
        }
      ],
      "color": "#D6A56D",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mkjcv1sb-ilohle",
      "name": "Path 10",
      "endPoint": {
        "x": 133.28795180722892,
        "y": 31.833734939759037,
        "heading": "linear",
        "reverse": false,
        "startDeg": 49,
        "endDeg": 49
      },
      "controlPoints": [
        {
          "x": 82.43012048192776,
          "y": 0.30963855421686404
        },
        {
          "x": 92.20843373493976,
          "y": 42.349397590361434
        }
      ],
      "color": "#8C96DD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "locked": false
    },
    {
      "id": "mkjcvoz8-jabp49",
      "name": "Path 11",
      "endPoint": {
        "x": 90,
        "y": 110,
        "heading": "linear",
        "reverse": false,
        "startDeg": 49,
        "endDeg": 35
      },
      "controlPoints": [],
      "color": "#67DB5A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-3za6r8pg3j5"
    },
    {
      "kind": "path",
      "lineId": "mkk2y7kl-a47zi3"
    },
    {
      "kind": "path",
      "lineId": "mkk2yfk3-v965h5"
    },
    {
      "kind": "path",
      "lineId": "mkjcnzkz-jjjotj"
    },
    {
      "kind": "path",
      "lineId": "mkjcp9wl-czdcgg"
    },
    {
      "kind": "path",
      "lineId": "mkjcv1sb-ilohle"
    },
    {
      "kind": "path",
      "lineId": "mkjcvoz8-jabp49"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-01-18T18:42:55.889Z"
}