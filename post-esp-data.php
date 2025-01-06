<?php
// Requerido para MySQL
$servername                       = "localhost";
$dbname                           = "esp_data";                 // Base de datos
$username                         = "root";                     // Usuario de MySQL
$password                         = "Laura_1sab3l";             // Clave de MySQL

$api_key_value                    = "tPmAT5Ab3j7F9";

$api_key= $t_colmena = $h_colmena = $t_ambiente = $h_ambiente = $presion = $luz = "";

if ($_SERVER["REQUEST_METHOD"] == "POST") {
    $api_key = test_input($_POST["api_key"]);
    if($api_key == $api_key_value) {
        $t_colmena      = test_input($_POST["t_colmena"]);
        $h_colmena      = test_input($_POST["h_colmena"]);
        $t_ambiente     = test_input($_POST["t_ambiente"]);
        $h_ambiente     = test_input($_POST["h_ambiente"]);
        $presion        = test_input($_POST["presion"]);
        $luz            = test_input($_POST["luz"]);
        
        $conn = new mysqli($servername, $username, $password, $dbname);
        if ($conn->connect_error) {
            die("Connection failed: " . $conn->connect_error);
        } 
        
        $sql = "INSERT INTO colmena 
        (t_colmena, 
        h_colmena, 
        t_ambiente, 
        h_ambiente, 
        presion, 
        luz)
        VALUES 
            ('" . $t_colmena . "',  '" 
            . $h_colmena . "', '" 
            . $t_ambiente . "', '" 
            . $h_ambiente . "', '" 
            . $presion . "', '" 
            . $luz . "')";
        
        if ($conn->query($sql) === TRUE) {
            echo "New record created successfully";
        } 
        else {
            echo "Error: " . $sql . "<br>" . $conn->error;
        }
    
        $conn->close();
    }
    else {
        echo "Wrong API Key provided.";
    }

}
else {
    echo "No data posted with HTTP POST.";
}

function test_input($data) {
    $data = trim($data);
    $data = stripslashes($data);
    $data = htmlspecialchars($data);
    return $data;
}