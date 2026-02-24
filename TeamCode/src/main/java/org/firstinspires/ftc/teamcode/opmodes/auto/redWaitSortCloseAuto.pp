{
  "startPoint": {
    "x": 129,
    "y": 115,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "mlzy7xpi-b3pcw1",
      "name": "scan motif",
      "endPoint": {
        "x": 86.8,
        "y": 88.2,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#86BCCB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzzo4jf-pn62kb",
      "endPoint": {
        "x": 100,
        "y": 72,
        "heading": "linear",
        "reverse": false,
        "startDeg": 90,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#DABAA9",
      "name": "positioning",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzzsqc6-5g3rt2",
      "endPoint": {
        "x": 131,
        "y": 72,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#6BCBA9",
      "name": "hit gate",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzzq2c8-eji8ll",
      "endPoint": {
        "x": 100,
        "y": 72,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
      },
      "controlPoints": [],
      "color": "#9D89A5",
      "name": "leave gate",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzzpjos-i6asa9",
      "endPoint": {
        "x": 86.8,
        "y": 88.2,
        "heading": "linear",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 43
      },
      "controlPoints": [],
      "color": "#78C79B",
      "name": "shoot preload",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzy8ola-0t45xd",
      "name": "intake second",
      "endPoint": {
        "x": 126.13,
        "y": 52,
        "heading": "linear",
        "reverse": false,
        "startDeg": 25,
        "endDeg": 0
      },
      "controlPoints": [
        {
          "x": 87.6,
          "y": 43
        }
      ],
      "color": "#BC8B89",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzy9onr-y07xee",
      "name": "shoot second",
      "endPoint": {
        "x": 86.8,
        "y": 88.2,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 43
      },
      "controlPoints": [
        {
          "x": 91.5,
          "y": 56
        }
      ],
      "color": "#9A6CAB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzynlqz-kz3psr",
      "endPoint": {
        "x": 122,
        "y": 84,
        "heading": "linear",
        "reverse": false,
        "startDeg": 25,
        "endDeg": 0
      },
      "controlPoints": [
        {
          "x": 100,
          "y": 79.5
        }
      ],
      "color": "#67BC96",
      "name": "intake first",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mlzye2ck-od2cip",
      "name": "shoot first",
      "endPoint": {
        "x": 87.79745,
        "y": 110.10889,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 20
      },
      "controlPoints": [],
      "color": "#AA8C67",
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
      "lineId": "mlzy7xpi-b3pcw1"
    },
    {
      "kind": "path",
      "lineId": "mlzzo4jf-pn62kb"
    },
    {
      "kind": "path",
      "lineId": "mlzzsqc6-5g3rt2"
    },
    {
      "kind": "path",
      "lineId": "mlzzq2c8-eji8ll"
    },
    {
      "kind": "path",
      "lineId": "mlzzpjos-i6asa9"
    },
    {
      "kind": "path",
      "lineId": "mlzy8ola-0t45xd"
    },
    {
      "kind": "path",
      "lineId": "mlzy9onr-y07xee"
    },
    {
      "kind": "path",
      "lineId": "mlzynlqz-kz3psr"
    },
    {
      "kind": "path",
      "lineId": "mlzye2ck-od2cip"
    }
  ],
  "version": "1.2.1",
  "timestamp": "2026-02-24T02:45:53.494Z"
}