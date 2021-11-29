window.onload = function () {
    //Check the support for the File API support
    if (window.File && window.FileReader && window.FileList && window.Blob) {
        
        fetch("robots.txt")
    }
    else {
        alert("Files are not supported");
    }
    } 