#!/bin/bash

echo "set_config_param window_rejection_enable 1" | nc os1-991837000025 7501
echo "reinitialize" | nc os1-991837000025 7501
