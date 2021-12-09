

$(document).ready(function(){
  $("button").click(function(){
    $("#div1").load("demo_test.txt");
  });
});

// register onload function
 document.getElementById('frmFile').onload = 
 readFile( "./weight.txt", utf, (err,data) =>{
   if(err) throw err;
   data.toString();
 })

