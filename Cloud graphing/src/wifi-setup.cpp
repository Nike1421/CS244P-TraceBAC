// #include <Arduino.h>
// #include <inttypes.h>
// #include <stdio.h>
// #include "esp_system.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "nvs.h"
// #include "nvs_flash.h"

// void setup()
// {
//   Serial.begin(9600);
//   delay(1000);

//   // Initialize NVS
//   esp_err_t err = nvs_flash_init();
//   if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
//   {
//     // NVS partition was truncated and needs to be erased
//     // Retry nvs_flash_init
//     ESP_ERROR_CHECK(nvs_flash_erase());
//     err = nvs_flash_init();
//   }

//   ESP_ERROR_CHECK(err);
  
//   Serial.printf("\n");
//   Serial.printf("Opening Non-Volatile Storage (NVS) handle... ");
  
//   nvs_handle_t my_handle;
  
//   err = nvs_open("storage", NVS_READWRITE, &my_handle);
//   if (err != ESP_OK)
//   {
//     Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
//   }
//   else
//   {
//     Serial.printf("Done\n");
//     // Write
//     Serial.printf("Updating ssid/pass in NVS ... ");
  
//     char ssid[] = "PhiRhoFun";
//     char pass[] = "September1984";
  
//     err = nvs_set_str(my_handle, "ssid", ssid);
//     err |= nvs_set_str(my_handle, "pass", pass);
  
//     Serial.printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
  
//     // Commit written value.
//     // After setting any values, nvs_commit() must be called to ensure changes
  
//     // are written to flash storage. Implementations may write to storage at
//     // other times, but this is not guaranteed.
  
//     Serial.printf("Committing updates in NVS ... ");
//     err = nvs_commit(my_handle);
//     Serial.printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
  
//     // Close
//     nvs_close(my_handle);
//   }
// }
// void loop()
// {
//   // put your main code here, to run repeatedly:
//   delay(1000);
// }