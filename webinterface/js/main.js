/*import { readFile } from './fs';

$(document).ready(function(){
  $("button").click(function(){
    $("#div1").load("demo_test.txt");
  });
});

// register onload function
 document.getElementById('frmFile').onload =
 readFile( "weight.txt", "utf-8", (err,data) =>{
   if(err) throw err;
   log.console(data.toString());
 })
*/
document.querySelector("txt_out").addEventListener(onload, async function() {
	try {
		let text_data = await downloadFile();
		document.querySelector("weight").textContent = text_data;
	}
	catch(e) {
		alert(e.message);
	}
});

async function downloadFile() {
	let response = await fetch("/weight.txt");
		
	if(response.status != 200) {
		throw new Error("Server Error");
	}
		
	// read response stream as text
	let text_data = await response.text();

	return text_data;
}
