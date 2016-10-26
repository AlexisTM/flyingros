var config = {
    paper : {
        offset:{
            x:50,
            y:50
        },
        width : 650 + 50,
        height : 650 + 50,
        least : 650,
        ratio : 1
    },
    ros : {
        ip : "130.104.97.95",
        safetyZone : {
            from : {
                x : 100,
                y : 100
            },
            to : {
                x : 500,
                y : 500
            }
        }
    }
}

var modalObject = {}

var type = {
    NULL : 0,
    INIT_UAV : 9,
    ARM : 13,
    DISARM : 11,
    LOITER : 121,
    TAKEOFF : 122,
    LAND : 123,
    TARGET : 124,
    GRAB : 193,
    TEST : 254,
}

var p = {
    dom : {},
    draw : {},
    o: {}
};

var dynamicData = {
    position : {
        x: 150,
        y: 150,
        z: 150
    },
    setpoint : {
        x: 150,
        y: 150,
        z: 150
    },
    currentObject : {}
}


var cache = {
    target : new ROSLIB.ServiceRequest({
        task: {
            name : "fromWeb",
            mission_type : type.TARGET,
            position : {
                x : 1.5,
                y : 1.5,
                z : 1.5
            },
            yaw : 0,
            data : [0.2,0.2,0.2]
        }
      })
}

var cmd = {};
var missionList = {};

window.onload = function() {
    ros_init();
    paper_init();
    dom_init();
    paper_resize();
    paper_update();
    setInterval(paper_update,100);
};


function dom_init(){
    // Task list
    document.getElementById("getMission").addEventListener("click", 
        function(){
            cmd.mission.get.callService({}, function(result){
                missionList.remove();
                var l = result.mission.tasks.length;
                var missionArray = [];
                for (var i = 0; i < l; i++) {
                    missionArray.push(taskForList(result.mission.tasks[i], i));
                }
                missionList.add(missionArray);
            });
        }); 

    var options = {
        valueNames: ['index', 'type', 'name', 'target', 'data', 'ID', 'bonus']
       // item : '<tr><td class="index"></td><td class="type"></td><td class="name"></td><td class="target"></td></tr>',
    };

    missionList = new List('Mission', options);

    modalObject.addTask = new Modalise('addTaskModal', {
      btnsOpen : [document.getElementById('addTask')]
    }).attach();

    modalObject.removeTask = new Modalise('removeModal', {
    }).attach().on('onConfirm', function(event){
      id = Number(modalObject.removeTask.modal.querySelector('.ID').innerHTML);
      
      if(!isFinite(id)) return false;

      console.log(id);

      var msg = new ROSLIB.Message({task: {
        ID : parseInt(id)
      }});
      
      cmd.task.remove.callService(msg, function(result){
        console.log(result);
      });
    });

    modalObject.removeTask.showData = function(task){
      modalObject.removeTask.modal.querySelector('.name').innerHTML = task._values.name
      modalObject.removeTask.modal.querySelector('.index').innerHTML = task._values.index
      modalObject.removeTask.modal.querySelector('.type').innerHTML = task._values.type
      modalObject.removeTask.modal.querySelector('.bonus').innerHTML = task._values.bonus
      modalObject.removeTask.modal.querySelector('.target').innerHTML = task._values.target
      modalObject.removeTask.modal.querySelector('.ID').innerHTML = task._values.ID
      modalObject.removeTask.modal.style.display = "block";
    }

    document.getElementById("clickTbody").addEventListener('click', function(e){
      task = getClickedObject(event);
      modalObject.removeTask.showData(task)
    })
}

function getClickedObject(event) {
    var tr = event.srcElement.closest('tr');
    var a = tr.getElementsByClassName('ID');
    if (a.length == 1) {
        var b = missionList.get('ID', a[0].innerText)
        if (b.length == 1) {
            // save data to use
            dynamicData.currentObject = b[0];
            return b[0];
        }
    }
    // Failed
    return false;
}

function ros_init(){
        //var cmd = {}
    var ros = new ROSLIB.Ros({
        url: 'ws://' + config.ros.ip + ':9090'
    });

    ros.on('connection', function() {
        console.info('Connected to the ROS server.');
    });

    ros.on('error', function(error) {
        console.error('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
        console.warn("Connection to ROS server closed.")
    });

    var reportListener = new ROSLIB.Topic({
        ros: ros,
        name: 'web/report',
        messageType: 'flyingros_msgs/Report'
    });

    var odomListener = new ROSLIB.Topic({
        ros: ros,
        name: 'gps/rtkfix',
        messageType: 'nav_msgs/Odometry'
    })

    var currentListener = new ROSLIB.Topic({
        ros: ros,
        name: 'flyingros/controller/task/current',
        messageType: 'flyingros_msgs/Task'
    })
    
    var taskAdd = new ROSLIB.Service({
        ros : ros,
        name : '/flyingros/controller/task/add',
        serviceType : 'flyingros_msgs/TaskHandle'
    });

    var taskRemove = new ROSLIB.Service({
        ros : ros,
        name : '/flyingros/controller/task/remove',
        serviceType : 'flyingros_msgs/TaskHandle'
    });

    var missionGet = new ROSLIB.Service({
        ros : ros,
        name : '/flyingros/controller/mission/get',
        serviceType : 'flyingros_msgs/MissionRequest'
    });

    reportListener.subscribe(function(message) {
        console.log(message);
    });

    odomListener.subscribe(function(message) {
        console.log(message);
    });

    currentListener.subscribe(function(message) {
        console.log(message);
    });

    cmd = {
        mission : {
            get : missionGet
        },
        task : {
            add : taskAdd,
            current : currentListener,
            remove : taskRemove
        },
        report: reportListener,
        odometry: odomListener
    };
}

function paper_init(){
    // Get a reference to the canvas object
    p.dom = document.getElementById('realtime');

    paper.setup(p.dom);

    p.o.origin = new paper.Point(config.paper.offset.x,config.paper.offset.y);
    p.o.xaxisExtremity = p.o.origin.add(650,0)
    p.o.yaxisExtremity = p.o.origin.add(0,650)

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
            meters = i/100
            tx.content = meters;
            tx.fontFamily = 'Courier New',
            tx.fontWeight = 'bold',
            tx.fontSize = 25

        var ty = new paper.PointText(p.o.origin.add(-20,10+i));
            ty.justification = 'center';
            ty.fillColor = 'black';
            meters = i/100
            ty.content = meters;
            ty.fontFamily = 'Courier New',
            ty.fontWeight = 'bold',
            ty.fontSize = 25

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
        tx.fontFamily = 'Courier New',
        tx.fontWeight = 'bold',
        tx.fontSize = 25

    var ty = new paper.PointText(p.o.origin.add(-20,i));
        ty.justification = 'center';
        ty.fillColor = 'black';
        ty.content = 'y(m)';
        ty.fontFamily = 'Courier New',
        ty.fontWeight = 'bold',
        ty.fontSize = 25
    p.o.axisData.push(tx,ty);

    p.o.UAV = new paper.Raster('app/drone.png', p.o.origin.add(150,150));
    p.o.setpoint = new paper.Raster('app/setpoint.png', p.o.origin.add(150,150));

    //p.o.clickable.onMouseMove = function(e){ console.log(e) }

    p.o.tool = new paper.Tool()

    p.o.tool.onMouseDown = function(e) {
        fromx = p.o.origin.x + config.ros.safetyZone.from.x;
        tox = p.o.origin.y + config.ros.safetyZone.to.x;
        fromy = p.o.origin.x + config.ros.safetyZone.from.y;
        toy = p.o.origin.y + config.ros.safetyZone.to.y;

        xp = limitClick(e.event.clientX, fromx, tox)
        yp = limitClick(e.event.clientY, fromy, toy)

        cache.target.task.position.x = (xp-p.o.origin.x)/100;
        cache.target.task.position.y = (yp-p.o.origin.y)/100;
        cache.target.task.yaw = 0;
        cache.target.task.data = [0.2,0.2,1];

        sendNewTask(cache.target.task);
        /*cmd.task.add.callService(cache.target, function(result){
            p.o.setpoint.position.x = xp;
            p.o.setpoint.position.y = yp;
        });*/
    }   
    paper.view.draw();
}

// limit values into a rectangle
function limitClick(data, from, to){
    return (data < from) ? from : (data > to) ? to : data; 
}

function paper_resize(){

    paper.view.draw();
}

function paper_update(){
    paper.view.draw();
    //console.warn("update")
}

function moveUAV(x,y,z,yaw){
    p.o.UAV.position = new paper.Point(250,250);
    p.o.UAV.rotation = yaw;
}


function idSpecificData(task){
    var data = { fa : '', bonus : ''}

    switch(task.mission_type){
        case type.NULL :
            data.fa =  '<i class="fa fa-question-circle" aria-hidden="true"></i>';
            break;
        case type.INIT_UAV :
            data.fa = '<i class="fa fa-list" aria-hidden="true"></i>';
            data.bonus = 'sleep : ' + Number(task.data[0]).toFixed(2) + 's';
            break;
        case type.ARM :
            data.fa = '<i class="fa fa-repeat" aria-hidden="true"></i><i class="fa fa-undo" aria-hidden="true"></i>';
            data.bonus = 'timeout : ' + Number(task.data[0]).toFixed(2) + 's';
            break;
        case type.DISARM :
            data.fa = '<i class="fa fa-stop" aria-hidden="true"></i>';
            data.bonus = 'timeout : ' + Number(task.data[0]).toFixed(2) + 's';
            break;
        case type.LOITER :
            data.fa = '<i class="fa fa-pause" aria-hidden="true"></i>';
            data.bonus = 'sleep : ' + Number(task.data[0]).toFixed(2) + 's';
            break;
        case type.TAKEOFF :
            data.fa = '<i class="fa fa-play" aria-hidden="true"></i>';
            data.bonus = 'precision Z : ' + Number(task.data[0]).toFixed(2) + 'm';
            break;
        case type.TARGET :
            data.fa = '<i class="fa fa-bullseye" aria-hidden="true"></i>';
            data.bonus = 'precision(' + Number(task.data[0]).toFixed(2) + ', ' + Number(task.data[0]).toFixed(2) + ', ' + Number(task.data[1]).toFixed(2) + ', ' + Number(task.data[2]).toFixed(2) + ')';
            break;
        case type.GRAB :
            data.fa = '<i class="fa fa-hand-rock-o" aria-hidden="true"></i>';
            data.bonus = Number(task.data[0]).toFixed(2) ? 'ON' : 'OFF';
            break;
        case type.TEST :
            data.fa = '<i class="fa fa-bomb" aria-hidden="true"></i>';
            break;
    }
    
    return data;
}

Array.prototype.last = function() {
    return this[this.length-1];
}

function sendNewTask(task){
  task.ID = 0;
  cmd.task.add.callService({task: task}, function(result){
              console.log(task);
              task.ID = Number(result.message);
              missionList.add([taskForList(task)])
            });
}

// Uncomplete task compliant
function taskForList(task, i) {
    i = isFinite(i) ? i : Number(missionList.items.last()._values.i);
    i = isFinite(i) ? i : missionList.items.length;
    specific = idSpecificData(task)
    return {
        type : task.mission_type ? specific.fa : specific.fa, 
        name : task.name || "NoName",
        target : task.position? "( " + task.position.x + ", " + task.position.y + ", " + task.position.z +")" : "NoTarget",
        index : i,
        ID : task.ID || "",
        data : task.data || "",
        bonus : specific.bonus}
}