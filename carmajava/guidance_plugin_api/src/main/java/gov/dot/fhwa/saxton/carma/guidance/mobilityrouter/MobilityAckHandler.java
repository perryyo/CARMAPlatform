/*
 * Copyright (C) 2018 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package gov.dot.fhwa.saxton.carma.guidance.mobilityrouter;

import cav_msgs.MobilityAck;

/**
 * Callback interface for handling inbound relevant {@link MobilityAck} messages
 */
public interface MobilityAckHandler {
  /**
   * Callback to be invoked upon receipt of a relevant {@link MobilityAck} message
   * 
   * @param msg The {@link MobilityAck} message that was received
   */
  void handleMobilityAckMessage(MobilityAck msg);
}