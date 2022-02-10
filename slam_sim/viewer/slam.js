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

const MAP_WIDTH = 1000;
const MAP_HEIGHT = 600;

const VIEW_WIDTH = 600;
const VIEW_HEIGHT = 600;
const VIEW_RADIUS = 290;

const HORIZ_SPACE = 20;   // Horizontal buffer between map and view.

const CANVAS_WIDTH = MAP_WIDTH + VIEW_WIDTH + 50;
const CANVAS_HEIGHT = MAP_HEIGHT;

const MAX_VISUAL_RANGE = VIEW_RADIUS - 5;


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
  fetchMap();
  setPosition();
  setTimeout(requestRedraw, 1000);
  setTimeout(requestRedraw, 3000);
  setTimeout(requestRedraw, 5000);
  //setInterval(requestRedraw, 1000);
}


function requestRedraw() {
    moveRobot();
    window.requestAnimationFrame(drawMap);
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
    drawRobot(ctx, 'red',
        viewData.senses.est_position.x, 
        viewData.senses.est_position.y,
        viewData.senses.est_position.heading_deg);
    drawRobot(ctx, 'black',
        viewData.position.x, 
        viewData.position.y,
        viewData.position.heading_deg);
  }
}


function drawRobot(ctx, color, x, y, ori_deg) {
  var s = Math.sin(D2R * ori_deg);
  var c = Math.cos(D2R * ori_deg);
  const width = 12.0;
  var dx = s * width;
  var dy = -c * width;
  //
  ctx.beginPath()
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  // Draw front
  ctx.moveTo(x, y);
  ctx.lineTo(x+dx, y+dy);
  // Draw left side
  ctx.moveTo(x, y);
  ctx.lineTo(x+dy/2, y-dx/2);
  ctx.lineTo(x+dy/2-dx, y-dx/2-dy);
  // Draw right side
  ctx.moveTo(x, y);
  ctx.lineTo(x-dy/2, y+dx/2);
  ctx.lineTo(x-dy/2-dx, y+dx/2-dy);
  ctx.stroke();
}


// Draw a dot on the canvas (i.e., a sensor data sample). Input x,y
//  in in robot's reference frame.
function drawDot(ctx, x, y) {
  //console.log("  dot at ", x, ",", y);
  ctx.beginPath()
  ctx.lineWidth = 2;
  ctx.strokeStyle = 'red';
  ctx.ellipse(x, y, 3, 3, 0, 0, 2*Math.PI);
  ctx.stroke();
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
  centerX = MAP_WIDTH + HORIZ_SPACE + VIEW_WIDTH/2;
  centerY = VIEW_WIDTH/2;
  radius = VIEW_WIDTH/2 - 10;  // slightly smaller than view
  // Draw bounds.
  ctx.beginPath()
  ctx.ellipse(centerX, centerY, radius, radius, 0, 0, 2*Math.PI);
  ctx.strokeStyle = 'black';
  ctx.lineWidth = 3;
  ctx.fillStyle = 'rgb(255, 224, 224)';
  ctx.fill();
  ctx.stroke();
  // Draw robot.
  drawRobot(ctx, 'black', centerX, centerY, 0);
  // Draw orientation marker.
  ori = viewData.position.heading_deg;
  ctx.beginPath();
  s = Math.sin(ori * D2R);
  c = Math.cos(ori * D2R);
  x = centerX + (VIEW_RADIUS-3) * s;
  y = centerY - (VIEW_RADIUS-3) * c;
  ctx.moveTo(x, y);
  x = centerX + (VIEW_RADIUS+10) * s;
  y = centerY - (VIEW_RADIUS+10) * c;
  ctx.lineTo(x, y);
  ctx.lineWidth = 10;
  ctx.strokeStyle = 'green';
  ctx.stroke();
  // Draw sensor data, from perspective of robot.
  for (let i=0; i<viewData.senses.lidar.length; i++) {
    sample = viewData.senses.lidar[i];
    range = sample.range;
    if ((range > MAX_VISUAL_RANGE) || (range < 0)) {
      continue;
    }
    bearing = sample.bearing;
    dx = range * Math.sin((ori-bearing) * D2R);
    dy = range * Math.cos((ori-bearing) * D2R);
    drawDot(ctx, centerX-dx, centerY-dy);
  }
}

init();

