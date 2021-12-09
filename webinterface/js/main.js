

$(document).ready(function(){
  $("button").click(function(){
    $("#div1").load("demo_test.txt");
  });
});
const fs = require('fs')
// register onload function
 document.getElementById('frmFile').onload =
 fs.readFile( "./weight.txt", "utf-8", (err,data) =>{
   if(err) throw err;
   log.console(data.toString());
 })

