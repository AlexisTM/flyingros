// paper

var cache = {
  target : new ROSLIB.ServiceRequest({
    task: {
      name : "fromWeb",
      mission_type : taskHelper.type.TARGET,
      position : {
        x : 1.5,
        y : 1.5,
        z : 1.5
      },
      yaw : 0,
      data : [0.2,0.2,0.2]
    }
  })
};

var p = {
  dom : {},
  draw : {},
  o: {}
};


// limit values into a rectangle
function limitClick(data, from, to){
  return (data < from) ? from : (data > to) ? to : data;
}

function paper_init(){
  // Get a reference to the canvas object
  p.dom = document.getElementById('realtime');

  paper.setup(p.dom);

  p.o.origin = new paper.Point(config.paper.offset.x,config.paper.offset.y);
  p.o.xaxisExtremity = p.o.origin.add(650,0);
  p.o.yaxisExtremity = p.o.origin.add(0,650);

  // Axis
  p.o.axisPath = new paper.Path();
  p.o.axisPath.strokeColor = 'black';
  p.o.axisPath.moveTo(p.o.xaxisExtremity);
  p.o.axisPath.lineTo(p.o.xaxisExtremity.add(-15,10));
  p.o.axisPath.lineTo(p.o.xaxisExtremity);
  p.o.axisPath.lineTo(p.o.xaxisExtremity.add(-15,-10));
  p.o.axisPath.lineTo(p.o.xaxisExtremity);
  p.o.axisPath.lineTo(p.o.origin);
  p.o.axisPath.lineTo(p.o.yaxisExtremity);
  p.o.axisPath.lineTo(p.o.yaxisExtremity.add(-10,-15));
  p.o.axisPath.lineTo(p.o.yaxisExtremity);
  p.o.axisPath.lineTo(p.o.yaxisExtremity.add(10,-15));


  p.o.originText = new paper.PointText(p.o.origin.add(0,-20));
  p.o.originText.justification = 'center';
  p.o.originText.fillColor = 'black';
  p.o.originText.content = '(0,0)';
  p.o.originText.fontFamily = 'Courier New';
  p.o.originText.fontWeight = 'bold';
  p.o.originText.fontSize = 25;

  p.o.axisData = [];

  var i;
  for(i = 100; i < 700; i = i + 100){
    var tx = new paper.PointText(p.o.origin.add(i,-20));
    tx.justification = 'center';
    tx.fillColor = 'black';
    meters = i/100;
    tx.content = meters;
    tx.fontFamily = 'Courier New';
    tx.fontWeight = 'bold';
    tx.fontSize = 25;

    var ty = new paper.PointText(p.o.origin.add(-20,10+i));
    ty.justification = 'center';
    ty.fillColor = 'black';
    meters = i/100;
    ty.content = meters;
    ty.fontFamily = 'Courier New';
    ty.fontWeight = 'bold';
    ty.fontSize = 25;

    var ly = new paper.Path.Line(p.o.origin.add(-5,i), p.o.origin.add(650,i))
    ly.strokeColor = 'black';
    ly.dashArray = [10, 4];

    var lx = new paper.Path.Line(p.o.origin.add(i,-5), p.o.origin.add(i,650))
    lx.strokeColor = 'black';
    lx.dashArray = [10, 4];

    p.o.axisData.push(tx,ty,lx,ly);
  }

  var tx = new paper.PointText(p.o.origin.add(i,-20));
  tx.justification = 'center';
  tx.fillColor = 'black';
  tx.content = 'x(m)';
  tx.fontFamily = 'Courier New';
  tx.fontWeight = 'bold';
  tx.fontSize = 25;

  var ty = new paper.PointText(p.o.origin.add(-20,i));
  ty.justification = 'center';
  ty.fillColor = 'black';
  ty.content = 'y(m)';
  ty.fontFamily = 'Courier New';
  ty.fontWeight = 'bold';
  ty.fontSize = 25
  p.o.axisData.push(tx,ty);

  p.o.UAV = new paper.Raster('static/drone.png', p.o.origin.add(150,150));
  p.o.setpoint = new paper.Raster('static/setpoint.png', p.o.origin.add(150,150));

  //p.o.clickable.onMouseMove = function(e){ console.log(e) }

  p.o.tool = new paper.Tool();

  p.o.tool.onMouseDown = function(e) {
    fromx = p.o.origin.x + config.ros.safetyZone.from.x;
    tox = p.o.origin.y + config.ros.safetyZone.to.x;
    fromy = p.o.origin.x + config.ros.safetyZone.from.y;
    toy = p.o.origin.y + config.ros.safetyZone.to.y;

    xp = limitClick(e.event.clientX, fromx, tox);
    yp = limitClick(e.event.clientY, fromy, toy);

    cache.target.task.position.x = (xp-p.o.origin.x)/100;
    cache.target.task.position.y = (yp-p.o.origin.y)/100;
    cache.target.task.yaw = 0;
    cache.target.task.data = [0.2,0.2,1];

    taskHelper.sendNew(cache.target.task);
  }
  paper.view.draw();
}

function moveUAV(x,y,z,yaw){
  p.o.UAV.position = new paper.Point(250,250);
  p.o.UAV.rotation = yaw;
}
