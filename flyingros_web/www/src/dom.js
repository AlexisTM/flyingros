// dom

var modal = {};
var mission = {};

modal.options = {
  valueNames: ['index', 'type', 'name', 'target', 'data', 'ID', 'bonus']
   // item : '<tr><td class="index"></td><td class="type"></td><td class="name"></td><td class="target"></td></tr>',
 };

 function SelectText(el, win) {
  win = win || window;
  var doc = win.document, sel, range;
  if (win.getSelection && doc.createRange) {
    sel = win.getSelection();
    range = doc.createRange();
    range.selectNodeContents(el);
    sel.removeAllRanges();
    sel.addRange(range);
  } else if (doc.body.createTextRange) {
    range = doc.body.createTextRange();
    range.moveToElementText(el);
    range.select();
  }
}

function syntaxHighlight(json) {
  if (typeof json != 'string') {
   json = JSON.stringify(json);
 }
 testVar = json;
 json = json.replace(/},{/g,'},<br>{')
 //json = json.replace(/&/g, '&amp;').replace(/\}\,\n[ ]+\{/g, '\}\, \{').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/\n/g, '<br>').replace(/\ /g, '&nbsp;&nbsp;');;
 return json.replace(/("(\\u[a-zA-Z0-9]{4}|\\[^u]|[^\\"])*"(\s*:)?|\b(true|false|null)\b|-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)/g, function (match) {
  var cls = 'number';
  var br = '';
  if (/^"/.test(match)) {
    if (/:$/.test(match)) {
      cls = 'key';
    } else {
      cls = 'string';
    }
  } else if (/true|false/.test(match)) {
    cls = 'boolean';
  } else if (/null/.test(match)) {
    cls = 'null';
  }
  return '<span class="' + cls + '">' + match + '</span>';
});
}

function show_mission_json(mission_json){
  mission.remove();
  var l = mission_json.tasks.length;
  var missionArray = [];
  for (var i = 0; i < l; i++) {
    missionArray.push(taskHelper.toList(mission_json.tasks[i], i));
  }
  mission.add(missionArray);
}

function dom_init() {
  mission = new List('Mission', modal.options);

  document.getElementById("getMission")
  .addEventListener("click", function(){
    cmd.mission.get.callService({}, function(result){
      show_mission_json(result.mission);
      dynamicData.currentTask = {}
    });
  });

  document.getElementById("removeMission").addEventListener("click", function(){
   cmd.mission.reset();
 });

  document.getElementById("selectTextExport").addEventListener("click", function(){
   SelectText(modal.exportMission.modal.querySelector('.exportData'), window);
 });

  modal.addTask = new Modalise('addTaskModal', {
    btnsOpen : [document.getElementById('addTask')]
  }).attach().on('onShow', function(){
    testVar = this;
    this.querySelector('.targetx').value = "";
    this.querySelector('.targety').value = "";
    this.querySelector('.targetz').value = "";
    this.querySelector('.name').value = "";
    this.querySelector('.data').value = "";
    this.querySelector('.yaw').value = "";
    this.querySelector('.type').selectedIndex = 0;
  }).on('onConfirm', function(){
    var mission_type = taskHelper.getMissionFromSelect(this.querySelector('.type'));
    var data = this.querySelector('.data').value;
    data = data ? JSON.parse(data) : [];
    data = Array.isArray(data) ? data : [];
    var taskToSend = {
      name : this.querySelector('.name').value,
      position : {
        x: Number(this.querySelector('.targetx').value),
        y: Number(this.querySelector('.targety').value),
        z: Number(this.querySelector('.targetz').value)
      },
      yaw : Number(this.querySelector('.yaw').value),
      data : data,
      ID : 0,
      mission_type : mission_type
    };

    taskHelper.sendNew(taskToSend);
  });

  modal.exportMission = new Modalise('exportModal', {
    btnsOpen : [document.getElementById('exportMission')]
  }).attach().on('onShow', function(){
    var textareajson = this.querySelector('.exportData');
    var downloadbutton = this.querySelector('.downloadJSON');
    cmd.mission.get.callService({}, function(result){
      // Simplifying useless data for more visibility
      for (var i = result.mission.tasks.length - 1; i >= 0; i--) {
        result.mission.tasks[i].ID = 0;
        result.mission.tasks[i].position.x = Number(result.mission.tasks[i].position.x.toFixed(3));
        result.mission.tasks[i].position.y = Number(result.mission.tasks[i].position.y.toFixed(3));
        result.mission.tasks[i].position.z = Number(result.mission.tasks[i].position.z.toFixed(3));
        for (var j = result.mission.tasks[i].data.length - 1; j >= 0; j--) {
          result.mission.tasks[i].data[j] = Number(result.mission.tasks[i].data[j].toFixed(3));
        }
      }

      textareajson.innerHTML = syntaxHighlight(result.mission);
      downloadbutton.href = 'data:application/octet-stream,' + encodeURIComponent(JSON.stringify(result.mission));
    });
  });

  modal.importMission = new Modalise('importModal', {
    btnsOpen : [document.getElementById('importMission')]
  }).attach().on('onShow', function(){
    this.querySelector('textarea').innerHTML = "";
  }).on('onConfirm', function(){
    // Reset the mission
    // When success : Add mission
    // When success : Get mission (with correct IDs)
    var json_mission = JSON.parse(this.querySelector('textarea').value);
    cmd.mission.reset(function(){
      cmd.mission.add.callService({mission: json_mission} ,function(result){
        cmd.mission.get.callService({}, function(result_mission){
          show_mission_json(result_mission.mission);
          dynamicData.currentTask = {}
        });
      });
    });
  });


  function handleFileSelect(evt) {
    evt.stopPropagation();
    evt.preventDefault();
    var files = evt.dataTransfer.files; // FileList object.
    reader = new FileReader();
    reader.onload = function(e){
      document.querySelector('textarea.dropfile').innerHTML = e.target.result;
    }
    for (var i = 0, f; f = files[i]; i++) {
      reader.readAsText(f);
    }
  }

  document.querySelector('textarea.dropfile').addEventListener('drop', handleFileSelect, false);

  modal.removeTask = new Modalise('removeModal', {
  }).attach().on('onConfirm', function(event){
    id = Number(modal.removeTask.modal.querySelector('.ID').innerHTML);

    if(!isFinite(id)) return false;

    cmd.task.remove.callService({task: {
      ID : parseInt(id)
    }}, function(result){
      mission.remove('ID', id);
    });
  });

  modal.removeTask.showData = function(task){
    modal.removeTask.modal.querySelector('.name').innerHTML = task._values.name
    modal.removeTask.modal.querySelector('.index').innerHTML = task._values.index
    modal.removeTask.modal.querySelector('.type').innerHTML = task._values.type
    modal.removeTask.modal.querySelector('.bonus').innerHTML = task._values.bonus
    modal.removeTask.modal.querySelector('.target').innerHTML = task._values.target
    modal.removeTask.modal.querySelector('.ID').innerHTML = task._values.ID
    modal.removeTask.modal.style.display = "block";
  };

  document.getElementById("clickTbody").addEventListener('click', function(e){
    task = getClicked(event);
    modal.removeTask.showData(task)
  });


  document.querySelector('input.alti').addEventListener('change', function(e){
    document.querySelector('label.alti').innerHTML = 'Altitude ' + e.srcElement.value + 'm';
  });
}

var getClicked = function (event) {
  var tr = event.srcElement.closest('tr');
  var a = tr.getElementsByClassName('ID');
  if (a.length == 1) {
    var b = mission.get('ID', a[0].innerText)
    if (b.length == 1) {
      // save data to use
      dynamicData.currentObject = b[0];
      return b[0];
    }
  }
// Failed
return false;
};
