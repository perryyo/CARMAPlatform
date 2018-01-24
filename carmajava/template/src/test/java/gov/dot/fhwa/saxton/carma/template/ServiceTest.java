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
import org.ros.node.NodeListener;
import org.ros.node.service.*;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Class for running integration and unit tests of the TransformServer node in the Roadway package
 */
public class ServiceTest extends RosTest {

  /**
   * Test of the get_transform service in the TransformServer node
   * Checks if the TransformServer node is subscribed to /tf and provides the get_transform service
   * The accuracy of the service is then tested
   */
  @Test public void testServiceAvailability() throws Exception {
    final NodeConfiguration secondConfig = NodeConfiguration.newPrivate(rosCore.getUri());
    final MessageFactory messageFactory = secondConfig.getTopicMessageFactory();
    final CountDownLatch countDownLatch = new CountDownLatch(1000);
    final String SERVICE_NAME = "test_service";
    final int NUM_THREADS = 10;

    // Create the anonymous node to test the server
    SaxtonBaseNode serverNode = new SaxtonBaseNode() {
      @Override public GraphName getDefaultNodeName() {
        return GraphName.of("server_node");
      }

      @Override public void onSaxtonStart(final ConnectedNode connectedNode) {

        // Setup Server
        ServiceServer<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> testServer =
         connectedNode.newServiceServer(SERVICE_NAME, GetDriversWithCapabilities._TYPE,
          new ServiceResponseBuilder<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse>() {
  
            @Override
            public void build(GetDriversWithCapabilitiesRequest req, GetDriversWithCapabilitiesResponse res) {
              res.setDriverData(req.getCapabilities());
            }
          });
      }

      @Override protected void handleException(Throwable e) {
        fail("Handle exception reached in test case");
      }
    };



    // Create the anonymous node to test the server
    SaxtonBaseNode clientNode = new SaxtonBaseNode() {
      List<ClientThread> threads = new LinkedList<>();
      @Override public GraphName getDefaultNodeName() {
        return GraphName.of("client_node");
      }

      @Override public void onSaxtonStart(final ConnectedNode connectedNode) {

        // Assert that the service was created
        ServiceClient<GetDriversWithCapabilitiesRequest, GetDriversWithCapabilitiesResponse> serviceClient =
          this.waitForService(SERVICE_NAME, GetDriversWithCapabilities._TYPE, connectedNode, 1000);
        assertNotNull("Service Client Not Null assertion", serviceClient);

        for(int i = 0; i < NUM_THREADS; i++) {
          ClientThread newThread = new ClientThread("Thread " + i, countDownLatch, messageFactory,
          SERVICE_NAME, serviceClient, connectedNode.getLog());
          newThread.start();
          threads.add(newThread);
        }
      }

      @Override protected void handleException(Throwable e) {
        for (ClientThread thread: threads) {
          thread.interrupt();
        }
        fail("Handle exception reached in test case");
      }

      @Override
      public void onShutdown(Node node) {
        for (ClientThread thread: threads) {
          thread.interrupt();
        }
      }
    };

    // Start the transform server node
    nodeMainExecutor.execute(serverNode, nodeConfiguration);
    // Start the anonymous node to test the server
    nodeMainExecutor.execute(clientNode, secondConfig);
    assertTrue(countDownLatch.await(20, TimeUnit.SECONDS)); // Check if service calls were successful
    // Shutdown nodes
    nodeMainExecutor.shutdownNodeMain(clientNode);
    // Shutting down the transform server from this test results in a exception on printing the service address
    nodeMainExecutor.shutdownNodeMain(serverNode);
    // Stack trace is automatically logged
    // ROS is shutdown automatically in cleanup from ROS Test
  }
}
