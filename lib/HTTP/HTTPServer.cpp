#include <esp_http_server.h>
#include <esp_log.h>
#include <time.h>
#include <map>
#include <vector>
#include <xbee_api.hpp>
#include <driver/uart.h>
#include <queue>
#include <tuple>
#include <iostream>
#include <sstream>
#include <cstdint>

static const char *TAG = "HTTP Server";

extern std::map<uint64_t, uint16_t> outletZigbeeAddresses;
extern std::queue<std::vector<uint8_t>> xbee_outgoing;
struct powerData
{
  float bP;
  float bPF;
  float tP;
  float tPF;
};
extern std::map<uint64_t, std::pair<uint64_t, powerData>> outletPowerDataSeconds;

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t top_on_handler(httpd_req_t *req)
{
  size_t buf_len;
  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128];
      esp_err_t result;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK)
      {
        ESP_LOGI(TAG, "Found URL query parameter => %s", param);
        json j = {
            {"op", 1},
            {"data", {{"value", 4}}}};
        std::string message = j.dump();
        std::string stringAddress = param;
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;
        std::vector<uint8_t> messageUART = formTXFrame(message, outletAddr, outletZigbeeAddresses[outletAddr], NULL, NULL);
        xbee_outgoing.push(messageUART);
        const char resp[] = "Top Outlet Turned On";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t top_off_handler(httpd_req_t *req)
{
  size_t buf_len;

  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128];
      esp_err_t result;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK)
      {
        json j = {
            {"op", 1},
            {"data", {{"value", 5}}}};

        std::string message = j.dump();
        std::string stringAddress = param;
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;
        std::vector<uint8_t> messageUART = formTXFrame(message, outletAddr, outletZigbeeAddresses[outletAddr], NULL, NULL);

        xbee_outgoing.push(messageUART);

        /* Send a simple response */
        const char resp[] = "Top Outlet Turned Off";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t bottom_on_handler(httpd_req_t *req)
{
  size_t buf_len;

  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128];
      esp_err_t result;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK)
      {
        json j = {
            {"op", 1},
            {"data", {{"value", 6}}}};

        std::string message = j.dump();
        std::string stringAddress = param;
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;
        std::vector<uint8_t> messageUART = formTXFrame(message, outletAddr, outletZigbeeAddresses[outletAddr], NULL, NULL);

        xbee_outgoing.push(messageUART);

        /* Send a simple response */
        const char resp[] = "Bottom Outlet Turned On";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t bottom_off_handler(httpd_req_t *req)
{
  size_t buf_len;
  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128];
      esp_err_t result;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK)
      {
        json j = {
            {"op", 1},
            {"data", {{"value", 7}}}};
        std::string message = j.dump();

        std::string stringAddress = param;
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;
        std::vector<uint8_t> messageUART = formTXFrame(message, outletAddr, outletZigbeeAddresses[outletAddr], NULL, NULL);

        xbee_outgoing.push(messageUART);

        /* Send a simple response */
        const char resp[] = "Bottom Outlet Turned Off";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t both_on_handler(httpd_req_t *req)
{
  size_t buf_len;
  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128];
      esp_err_t result;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK)
      {
        json j = {
            {"op", 1},
            {"data", {{"value", 3}}}};

        std::string message = j.dump();

        std::string stringAddress = param;
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;
        std::vector<uint8_t> messageUART = formTXFrame(message, outletAddr, outletZigbeeAddresses[outletAddr], NULL, NULL);

        xbee_outgoing.push(messageUART);

        /* Send a simple response */
        const char resp[] = "Both on";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t both_off_handler(httpd_req_t *req)
{
  size_t buf_len;
  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128];
      esp_err_t result;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK)
      {
        json j = {
            {"op", 1},
            {"data", {{"value", 0}}}};

        std::string message = j.dump();
        std::string stringAddress = param;
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;
        std::vector<uint8_t> messageUART = formTXFrame(message, outletAddr, outletZigbeeAddresses[outletAddr], NULL, NULL);
        xbee_outgoing.push(messageUART);
        /* Send a simple response */
        const char resp[] = "Both Off";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t both_power_handler(httpd_req_t *req)
{
  size_t buf_len;
  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128];
      esp_err_t result;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK)
      {
        std::string stringAddress = param;
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;
        std::pair<uint64_t, powerData> firstMeasurement = outletPowerDataSeconds[outletAddr];
        
        json j = {
        {"time", firstMeasurement.first},
        {"bottomPower", firstMeasurement.second.bP},
        {"topPower", firstMeasurement.second.tP},
        {"bottomPF", firstMeasurement.second.bPF},
        {"topPF", firstMeasurement.second.tPF}};

        std::string httpResponse = j.dump();

        /* Send a simple response */
        // const char resp[] = httpResponse;
        httpd_resp_send(req, httpResponse.c_str(), HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

//  URI Handler functions
/* Our URI handler function to be called during GET /uri request */
esp_err_t all_outlets(httpd_req_t *req)
{
  // json j = outletZigbeeAddresses;

  json message = {};

  for (auto outlet : outletZigbeeAddresses)
  {
    json j = {
        {"Name", ""},
        {"longAddress", outlet.first},
        {"shortAddress", outlet.second}};
    message += j;
  }

  std::string httpResponse = message.dump();
  // std::cout << "Server: " << httpResponse << std::endl;
  /* Send a simple response */
  // const char resp[] = httpResponse;
  httpd_resp_send(req, httpResponse.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

esp_err_t power_limit(httpd_req_t *req)
{
  size_t buf_len;
  /* Read URL query string length and allocate memory for length + 1,
   * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1)
  {
    char *buf = (char *)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
    {
      ESP_LOGI(TAG, "Found URL query => %s", buf);
      char param[128*2];
      char powerLim[128];
      char c_duration[128];
      esp_err_t result;
      esp_err_t result2;
      esp_err_t result3;
      result = httpd_query_key_value(buf, "Address", param, sizeof(param));
      result2 = httpd_query_key_value(buf, "PLim", powerLim, sizeof(param));
      result3 = httpd_query_key_value(buf, "Dur", c_duration, sizeof(c_duration));
      free(buf);
      /* Get value of expected key from query string */
      if (result == ESP_OK && result2 == ESP_OK && result3 == ESP_OK)
      {
        ESP_LOGI(TAG, "Found URL query parameter => %s", param);
        ESP_LOGI(TAG, "Found URL query parameter => %s", powerLim);
        ESP_LOGI(TAG, "Found URL query parameter => %s", c_duration);


        float pLim = std::stof(powerLim);
        int dur =std::stoi(c_duration);
        uint64_t outletAddr;
        std::istringstream iss(param);
        iss >> outletAddr;

        // TODO: Format for power limit
        json j = {
            {"op", 3},
            {"data", {{"value", pLim}, {"duration", dur}}}};
        std::string message = j.dump();
        std::cout << "8"<< std::endl;
        std::vector<uint8_t> messageUART = formTXFrame(message, outletAddr, outletZigbeeAddresses[outletAddr], NULL, NULL);
        xbee_outgoing.push(messageUART);
        const char resp[] = "Power Limit Sent";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
      }
      else
      {
        std::cout << "Result is: " << result << std::endl;
        return ESP_FAIL;
      }
    }
    free(buf);
  }
  return ESP_FAIL;
}

/* Our URI handler function to be called during POST /uri request */
esp_err_t post_handler(httpd_req_t *req)
{
  /* Destination buffer for content of HTTP POST request.
   * httpd_req_recv() accepts char* only, but content could
   * as well be any binary data (needs type casting).
   * In case of string data, null termination will be absent, and
   * content length would give length of string */
  char content[100];

  /* Truncate if content length larger than the buffer */
  size_t recv_size = req->content_len > sizeof(content) ? sizeof(content) : req->content_len; // MIN(req->content_len, sizeof(content));

  int ret = httpd_req_recv(req, content, recv_size);
  ESP_LOGI(TAG, "Post message: %s", content);
  if (ret <= 0)
  { /* 0 return value indicates connection closed */
    /* Check if timeout occurred */
    if (ret == HTTPD_SOCK_ERR_TIMEOUT)
    {
      /* In case of timeout one can choose to retry calling
       * httpd_req_recv(), but to keep it simple, here we
       * respond with an HTTP 408 (Request Timeout) error */
      httpd_resp_send_408(req);
    }
    /* In case of error, returning ESP_FAIL will
     * ensure that the underlying socket is closed */
    return ESP_FAIL;
  }

  /* Send a simple response */
  const char resp[] = "URI POST Response";
  httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

/* URI handler structure for top receptacle on */
httpd_uri_t uri_top_on = {
    .uri = "/top/on",
    .method = HTTP_GET,
    .handler = top_on_handler,
    .user_ctx = NULL};

/* URI handler structure for top receptacle off */
httpd_uri_t uri_top_off = {
    .uri = "/top/off",
    .method = HTTP_GET,
    .handler = top_off_handler,
    .user_ctx = NULL};

/* URI handler structure for bottom receptacle on */
httpd_uri_t uri_bottom_on = {
    .uri = "/bottom/on",
    .method = HTTP_GET,
    .handler = bottom_on_handler,
    .user_ctx = NULL};

/* URI handler structure for  bottom receptacle off */
httpd_uri_t uri_bottom_off = {
    .uri = "/bottom/off",
    .method = HTTP_GET,
    .handler = bottom_off_handler,
    .user_ctx = NULL};

/* URI handler structure for  bottom receptacle off */
httpd_uri_t uri_both_on = {
    .uri = "/both/on",
    .method = HTTP_GET,
    .handler = both_on_handler,
    .user_ctx = NULL};

/* URI handler structure for  bottom receptacle off */
httpd_uri_t uri_both_off = {
    .uri = "/both/off",
    .method = HTTP_GET,
    .handler = both_off_handler,
    .user_ctx = NULL};

/* URI handler structure for top receptacle on */
httpd_uri_t uri_both_power = {
    .uri = "/both/power",
    .method = HTTP_GET,
    .handler = both_power_handler,
    .user_ctx = NULL};

/* URI handler structure for top receptacle on */
httpd_uri_t uri_all_outlets = {
    .uri = "/allOutlets",
    .method = HTTP_GET,
    .handler = all_outlets,
    .user_ctx = NULL};

/* URI handler structure for top receptacle on */
httpd_uri_t uri_pow_Lim = {
    .uri = "/powerLimit",
    .method = HTTP_GET,
    .handler = power_limit,
    .user_ctx = NULL};

/* URI handler structure for POST /uri */
httpd_uri_t uri_post = {
    .uri = "/uri",
    .method = HTTP_POST,
    .handler = post_handler,
    .user_ctx = NULL};

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
  /* Generate default configuration */
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.core_id = 0;
  config.server_port = 80;
  config.ctrl_port = 5555;
  config.max_uri_handlers = 12;

  /* Empty handle to esp_http_server */
  httpd_handle_t server = NULL;

  /* Start the httpd server */
  if (httpd_start(&server, &config) == ESP_OK)
  {
    /* Register URI handlers */
    httpd_register_uri_handler(server, &uri_top_on);
    httpd_register_uri_handler(server, &uri_top_off);
    httpd_register_uri_handler(server, &uri_bottom_on);
    httpd_register_uri_handler(server, &uri_bottom_off);
    httpd_register_uri_handler(server, &uri_both_on);
    httpd_register_uri_handler(server, &uri_both_off);
    httpd_register_uri_handler(server, &uri_both_power);
    httpd_register_uri_handler(server, &uri_all_outlets);
    httpd_register_uri_handler(server, &uri_pow_Lim);
    // httpd_register_uri_handler(server, &uri_name_outlet);

    httpd_register_uri_handler(server, &uri_post);
  }
  /* If server failed to start, handle will be NULL */
  return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server)
{
  if (server)
  {
    /* Stop the httpd server */
    httpd_stop(server);
  }
}