package gov.dot.fhwa.saxton.carma.template;

import cav_srvs.GetDriversWithCapabilities;
import cav_srvs.GetDriversWithCapabilitiesRequest;
import cav_srvs.GetDriversWithCapabilitiesResponse;
import gov.dot.fhwa.saxton.carma.rosutils.RosTest;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.service.*;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.apache.commons.logging.Log;
import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.fail;

/**
 * Class for making service requests
 */
public class ClientThread extends Thread {
  final String requestStr;
  final CountDownLatch countDownLatch;
  final MessageFactory messageFactory;
  final ServiceClient<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> serviceClient;
  final String service;
  final boolean[] done = new boolean[1];
  final Log log;

  ClientThread(String requestStr, CountDownLatch countDownLatch, MessageFactory messageFactory,
   String service, ServiceClient<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> serviceClient, Log log) {
    this.requestStr = requestStr;
    this.countDownLatch = countDownLatch;
    this.messageFactory = messageFactory;
    this.service = service;
    this.serviceClient = serviceClient;
    this.done[0] = true;
    this.log = log;
  }

  @Override
  public void run(){  
    // Build ros messages
    if (done[0]) {
      log.info("\n\n In If \n\n");
      final List<String> caps = new LinkedList<>();
      caps.add(requestStr);
      done[0] = false;
      GetDriversWithCapabilitiesRequest req =
      messageFactory.newFromType(GetDriversWithCapabilitiesRequest._TYPE);
    
      req.setCapabilities(caps);

      serviceClient.call(req, new ServiceResponseListener<GetDriversWithCapabilitiesResponse>() {
        @Override public void onSuccess(GetDriversWithCapabilitiesResponse response) {
          log.info("\n\n Making Request \n\n");
          // Compare the requested transform with the
          log.info("Request: " + req.getCapabilities().get(0) + " Result: " + response.getDriverData().get(0));
          assertTrue(response.getDriverData().get(0).equals(req.getCapabilities().get(0)));
          countDownLatch.countDown();
          done[0] = true;
        }

        @Override
        public void onFailure(RemoteException e) {
          fail("Service request failed for request " + requestStr);
        }
      });
    }
  }  
}