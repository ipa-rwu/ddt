function disable_button(id) {
  var element = document.getElementById(id);
  var green = "rgb(4, 170, 109)"; //rgba or rgb, so if you really want to use element, you need use regexp.
  var grey = "rgb(105, 107, 105)"; //pls change your css to element too.
  if (id == "show_graph_button"){
    other_name = "pause_graph_button"
  }
  else{
    other_name = "show_graph_button"
  }
  console.log(element);
  var b = document.getElementById(other_name);
  if (element.value == "1"){
    if (id == "show_graph_button"){
      flag_show = true
    }
    element.style.backgroundColor = grey;
    element.value = "0";
    console.log("change to grey");
    b.value  = "1";
    // b.style.setProperty('background-color', green);
    b.style.backgroundColor = green
  }
}
