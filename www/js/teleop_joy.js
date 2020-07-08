// --- for teleop  ---
if(!Twist){
    var Twist = {
        ros : null,
        name : "",
        init : function(){
            this.ros = new ROSLIB.Ros();
            this.ros.on('error', function(error) {
                document.getElementById('state').innerHTML = "Error";
            });
            this.ros.on('connection', function(error) {
                document.getElementById('state').innerHTML = "Connect";
            });
            this.ros.on('close', function(error) {
                document.getElementById('state').innerHTML = "Close";
            });
            this.ros.connect('ws://' + location.hostname + ':9090');
        },
        send : function(linearX, angularZ){
            var pub = new ROSLIB.Topic({
                ros : this.ros,
                name : '/cmd_vel',
                messageType : 'geometry_msgs/Twist'
            });
            var twist = new ROSLIB.Message({
              linear:{x:linearX, y:0.0, z:0.0}, 
              angular:{x:0.0, y:0.0, z:angularZ}
            });
            pub.publish(twist);
        }
    }
    // Twist.init();

    window.onload = function(){
    };
    window.onunload = function(){
        Twist.ros.close();
    };
}

//position -1.0~1.0
last_X=0.0;
last_Y=0.0;
last_enable=false;

window.onload=function(){
  canvas=$('canvas');
  expandCanvas();
  reticle();
}

function expandCanvas(){
  var b = document.body;
  var d = document.documentElement;
  canvas.width  = 400;
  canvas.height = 400;
}

canvas.onmousedown = function(e){
  last_X=(e.pageX - canvas.offsetLeft - 200) / 200;
  last_Y=(e.pageY - canvas.offsetTop  - 200) / 200;
  last_enable=true;
}

canvas.onmousemove = function(e){
  last_X=(e.pageX - canvas.offsetLeft - 200) / 200;
  last_Y=(e.pageY - canvas.offsetTop  - 200) / 200;
}

canvas.onmouseup=function(e){
  last_X=0.0
  last_Y=0.0
  last_enable=false;
  Twist.send(0.0, 0.0);
}

function reticle(){
  var target=$("canvas");
  var context=target.getContext('2d');
  center_x=200;
  center_y=200;
  line_a=10;
  line_b=100;

  context.beginPath();
  context.moveTo(center_x+line_a, center_y);
  context.lineTo(center_x+line_b, center_y);
  context.closePath();
  context.stroke();

  context.beginPath();
  context.moveTo(center_x-line_a, center_y);
  context.lineTo(center_x-line_b, center_y);
  context.closePath();
  context.stroke();

  context.beginPath();
  context.moveTo(center_x, center_y+line_a);
  context.lineTo(center_x, center_y+line_b);
  context.closePath();
  context.stroke();

  context.beginPath();
  context.moveTo(center_x, center_y-line_a);
  context.lineTo(center_x, center_y-line_b);
  context.closePath();
  context.stroke();
}

function interval_process(){
  if(last_enable){
    var linearX  = -0.5*last_Y;
    var angularZ = -1.0*last_X;

    document.getElementById("linearX" ).textContent = "linearX: " +linearX +" m/s" ;
    document.getElementById("angularZ").textContent = "angularZ: "+angularZ+" rad/s" ;
    console.log(linearX, angularZ);
    Twist.send(linearX, angularZ);
  }
}
setInterval("interval_process()",100);

function $(id){
  return document.getElementById(id);
}
//--------------------