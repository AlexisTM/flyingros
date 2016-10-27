// dom

var modal = {};
var mission = {};

modal.options = {
  valueNames: ['index', 'type', 'name', 'target', 'data', 'ID', 'bonus']
   // item : '<tr><td class="index"></td><td class="type"></td><td class="name"></td><td class="target"></td></tr>',
 };

function dom_init() {

  mission = new List('Mission', modal.options);

  document.getElementById("getMission")
  .addEventListener("click",
                    function(){
                      cmd.mission.get.callService({}, function(result){
                        mission.remove();
                        var l = result.mission.tasks.length;
                        var missionArray = [];
                        for (var i = 0; i < l; i++) {
                          missionArray.push(taskHelper.toList(result.mission.tasks[i], i));
                        }
                        mission.add(missionArray);
                      });
                    });

  modal.addTask = new Modalise('addTaskModal', {
    btnsOpen : [document.getElementById('addTask')]
  }).attach();

  modal.removeTask = new Modalise('removeModal', {
  })
  .attach().on('onConfirm', function(event){
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
