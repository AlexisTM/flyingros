function circle(origin = {x : 3, y:3}, r=2, n=16, altitude=2.5){
  result = [];
  for (var i = 0; i < n; i++) {
    x = Math.cos(2*Math.PI*i/n)*r;
    y = Math.cos(2*Math.PI*i/n)*r;
    result.push({"name":"Go to","yaw":0,"position":{"x":(origin.x+x),"y":(origin.y+y),"z":altitude},"data":[0.2,0.2,1],"ID":0,"mission_type":124});
    result.push({"name":"Wait a bit","yaw":0,"position":{"y":0,"x":0,"z":0},"data":[3],"ID":0,"mission_type":121});
  }
  return result;
}

console.log(JSON.stringify(circle()))