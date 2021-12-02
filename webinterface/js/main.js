//import weight from "./weight.json";

//document.getElementById('weight').innerHTML = JSON.parse(weight);

 document.getElementById('weight').fetch("./weight.json").then(function(resp){
     return resp.json();
 }).then(function(data){console.log(data);});
