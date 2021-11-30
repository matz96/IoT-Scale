const readline = require('readline');
const fs = require('fs');

var file = '/var/www/html/pro3e/webinterfac/weight.txt';
var rl = readLine.createInterface({
    input : f.createReadStream(file),
    output : process.stdout,
    terminal: false
});
rl.on('line', function (text) {
 console.log(text);
});