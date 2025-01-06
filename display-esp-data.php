<!DOCTYPE html>
<html><body>
<h1>Datos de la colmena</h1>
<?php

// Requerido para MySQL
$servername                       = "localhost";
$dbname                           = "esp_data";                 // Base de datos
$username                         = "root";                     // Usuario de MySQL
$password                         = "Laura_1sab3l";             // Clave de MySQL

// Create connection
$conn = new mysqli($servername, $username, $password, $dbname);
// Check connection
if ($conn->connect_error) {
    die("Connection failed: " . $conn->connect_error);
} 

$sql = "SELECT  
  id, 
  hora, 
  t_colmena, 
  h_colmena, 
  t_ambiente, 
  h_ambiente,
  presion,
  luz
    FROM colmena ORDER BY id DESC";

echo '<table cellspacing="5" cellpadding="5">
      <tr> 
        <td>ID</td> 
        <td>Hora</td> 
        <td>Temperatura Colmena</td> 
        <td>Humedad Colmena</td> 
        <td>Temperatura Ambiente</td>
        <td>Humedad Ambiente</td> 
        <td>Presion atmosferica</td> 
        <td>Indice luminico</td> 
      </tr>';
 
if ($result = $conn->query($sql)) {
    while ($row = $result->fetch_assoc()) {
        $row_id         = $row["id"];
        $row_hora       = $row["hora"];
        $row_t_colmena  = $row["t_colmena"];
        $row_h_colmena  = $row["h_colmena"];
        $row_t_ambiente = $row["t_ambiente"]; 
        $row_h_ambiente = $row["h_ambiente"]; 
        $row_presion    = $row["presion"];
        $row_luz        = $row["luz"];

        // Uncomment to set timezone to - 1 hour (you can change 1 to any number)
        //$row_reading_time = date("Y-m-d H:i:s", strtotime("$row_reading_time - 1 hours"));
      
        // Uncomment to set timezone to + 4 hours (you can change 4 to any number)
        //$row_reading_time = date("Y-m-d H:i:s", strtotime("$row_reading_time + 4 hours"));
      
        echo '<tr> 
                <td>' . $row_id           .         '</td> 
                <td>' . $row_hora         .         '</td> 
                <td>' . $row_t_colmena    .         '</td> 
                <td>' . $row_h_colmena    .         '</td> 
                <td>' . $row_t_ambiente   .         '</td>
                <td>' . $row_h_ambiente   .         '</td> 
                <td>' . $row_presion      .         '</td> 
                <td>' . $row_luz          .         '</td> 
              </tr>';
    }
    $result->free();
}

$conn->close();
?> 
</table>
</body>
</html>