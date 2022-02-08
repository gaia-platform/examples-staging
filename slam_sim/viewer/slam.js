// Script for viewing slam data.
// The satellite is pinged every X msec to get the most recent
//  data to display.
// TODO add interface/logic to control robot from python and have that
//  be used to control updates. OR... add some JS controls.

////////////////////////////////////////////////////////////////////////
// satellite interface

mapUrl = 'http://0.0.0.0:4501/get-map/';
posUrl = 'http://0.0.0.0:4501/position/';
moveUrl = 'http://0.0.0.0:4501/motion/';

// Stores information returned from satellite.
worldMap = null;    // Map of world.
viewData = null;    // View from robot.


const R2D = 180.0 / Math.PI;
const D2R = Math.PI / 180.0;

const MAX_VISUAL_RANGE = 295.0;

var estimatedImg = new Image();
var actualImg = new Image();

// hit satellite's endpoint to get latest data
function fetchMap() {
  fetch(mapUrl, { method: "GET", mode: 'cors', headers: {} })
    .then(response => response.json())
    .then(data =>  { if (data != null) worldMap = data; });
  console.log("Requested map");
}; 


const posCmd = { x: 250.0, y: 250.0, heading: 315 };

function setPosition() {
  fetch(posUrl, { method: "POST", mode: 'cors', 
      headers: { 'Content-Type': 'application/json', },
      body: JSON.stringify(posCmd)})
    .then(response => response.json())
    .then(data =>  { if (data != null) viewData = data; });
  console.log("Sent position command");
}; 


const moveCmd = { distance: 5.0, turn_angle: 1.0 };

function moveRobot() {
  fetch(moveUrl, { method: "POST", mode: 'cors', 
      headers: { 'Content-Type': 'application/json', },
      body: JSON.stringify(moveCmd)})
    .then(response => response.json())
    .then(data =>  { if (data != null) viewData = data; });
  console.log("Sent motion command");
}; 


////////////////////////////////////////////////////////////////////////
// main script interface

function init() {
  estimatedImg.src = "images/estimated.png";
  actualImg.src = "images/actual.png";
  fetchMap();
  setPosition();
  //setTimeout(requestRedraw, 1000);
  //setTimeout(requestRedraw, 2000);
  //setTimeout(requestRedraw, 3000);
  setInterval(requestRedraw, 1000);
}


function requestRedraw() {
    moveRobot();
    window.requestAnimationFrame(drawMap);
}


var MAP_WIDTH = 1000;
var MAP_HEIGHT = 600;

var VIEW_WIDTH = 600;
var VIEW_HEIGHT = 600;

var HORIZ_SPACE = 20;   // Horizontal buffer between map and view.

var CANVAS_WIDTH = MAP_WIDTH + VIEW_WIDTH + 50;
var CANVAS_HEIGHT = MAP_HEIGHT;


function drawImageCentered(ctx, img, x, y) {
  console.log("Image centered at ", x, ", ", y);
  ctx.drawImage(img, x-img.width/2, y-img.height/2);
}

function rotateAndPaintImage(ctx, image, rotDeg, posX, posY) {
  console.log("Rotated image: ", posX, ",", posY, " : ", rotDeg);
  rotRad = rotDeg * D2R;
  ctx.translate(posX-image.width/2, posY-image.height/2);
  ctx.rotate(rotRad);
  drawImageCentered(ctx, image, 0, 0);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
}


function drawMap() {
  if ((worldMap == null) || (worldMap.status != "ready")) {
    fetchMap();
    return;
  }
  console.log("Drawing map");
  var ctx = document.getElementById('canvas1').getContext('2d');
  // Draw map.
  ctx.clearRect(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT);
  ctx.fillStyle = 'rgb(224, 224, 255)';
  ctx.fillRect(0, 0, MAP_WIDTH, MAP_HEIGHT);
  ctx.strokeStyle = 'black';
  objects = worldMap.world;
  console.log(objects);
  for (let i=0; i<worldMap.world.length; i++) {
    obj = worldMap.world[i];
    console.log("  ", obj.name);
    ctx.beginPath();
    ctx.moveTo(obj.vertices[0].x, obj.vertices[0].y);
    for (let j=1; j<obj.vertices.length; j++) {
      ctx.lineTo(obj.vertices[j].x, obj.vertices[j].y);
    }
    ctx.lineWidth = 3;
    ctx.stroke();
  }
  drawSensors(ctx);
  // Draw robot at correct orieintation.
  if (viewData != null) {
    rotateAndPaintImage(ctx, estimatedImg, 
        viewData.senses.est_position.heading_deg, 
        viewData.senses.est_position.x, 
        viewData.senses.est_position.y);
    rotateAndPaintImage(ctx, actualImg, viewData.position.heading_deg, 
        viewData.position.x, viewData.position.y);
  }
}


// Draw a dot on the canvas (i.e., a sensor data sample). Input x,y
//  in in robot's reference frame.
function drawDot(ctx, xOff, x, y) {
  //console.log("  dot at ", x, ",", y);
  dispX = xOff + VIEW_WIDTH/2 + x;
  dispY = VIEW_HEIGHT/2 - y;
  ctx.beginPath()
  ctx.lineWidth = 2;
  ctx.strokeStyle = 'red';
  ctx.ellipse(dispX, dispY, 3, 3, 0, 0, 2*Math.PI);
  ctx.stroke();
      //VIEW_WIDTH/2-2, VIEW_HEIGHT/2-2, 0, 0, 2*Math.PI);
//  ctx.beginPath();
//  ctx.lineWidth = 3;
//  ctx.fillStyle = 'black';
//  ctx.moveTo(1300, 0);
//  ctx.lineTo(1300, 600);
////  ctx.moveTo(dispX-2, dispY);
////  ctx.lineTo(dispX+2, dispY);
//  console.log("  ", (dispX-2), ", ", dispY, "  : ", (dispX+2));
//  ctx.stroke;
//  ctx.moveTo(dispX, dispY-2);
//  ctx.lineTo(dispX, dispY+2);
//  ctx.stroke;
}


function drawSensors(ctx) {
  console.log("Drawing sensors");
  if (viewData == null) {
    console.log("No view data");
    setPosition();
    return;
  }
  console.log(viewData);
  xOff = MAP_WIDTH + HORIZ_SPACE;
  // Draw bounds.
  ctx.beginPath()
  ctx.ellipse(xOff+VIEW_WIDTH/2, VIEW_HEIGHT/2, 
      VIEW_WIDTH/2-2, VIEW_HEIGHT/2-2, 0, 0, 2*Math.PI);
  ctx.strokeStyle = 'black';
  ctx.lineWidth = 3;
  ctx.fillStyle = 'rgb(255, 224, 224)';
  ctx.fill();
  ctx.stroke();
  // Draw robot.
  drawImageCentered(ctx, actualImg, xOff+VIEW_WIDTH/2, VIEW_HEIGHT/2);
  //ctx.drawImage(estimatedImg, xOff+VIEW_WIDTH/2-estimatedImg.width/2, VIEW_HEIGHT/2-estimatedImg.height/2);
  // TODO Draw sensor data.
  for (let i=0; i<viewData.senses.lidar.length; i++) {
    sample = viewData.senses.lidar[i];
    range = sample.range;
    if ((range > MAX_VISUAL_RANGE) || (range < 0)) {
      continue;
    }
    bearing = sample.bearing;
    dx = range * Math.sin(bearing * D2R);
    dy = range * Math.cos(bearing * D2R);
    drawDot(ctx, xOff, dx, dy);
  }
}

init();

