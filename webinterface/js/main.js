import { readFile } from 'fs';

$(document).ready(function(){
  $("button").click(function(){
    $("#div1").load("demo_test.txt");
  });
});

// register onload function
 document.getElementById('frmFile').onload =
 readFile( "./weight.txt", "utf-8", (err,data) =>{
   if(err) throw err;
   log.console(data.toString());
 })

