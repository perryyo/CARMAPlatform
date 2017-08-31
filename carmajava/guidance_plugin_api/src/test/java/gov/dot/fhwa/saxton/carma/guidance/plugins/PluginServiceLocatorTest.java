package gov.dot.fhwa.saxton.carma.guidance.plugins;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.pubsub.IPubSubService;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class PluginServiceLocatorTest {
    @Before
    public void setUp() {
        as = mock(ArbitratorService.class);
        pms = mock(PluginManagementService.class);
        pss = mock(IPubSubService.class);

        psl = new PluginServiceLocator(as, pms, pss);
    }

    @After
    public void tearDown() {
        as = null;
        pms = null;
        pss = null;
        psl = null;
    }

    @Test public void getArbitratorService() throws Exception {
        assertEquals(as, psl.getArbitratorService());
    }

    @Test public void getPluginManagementService() throws Exception {
        assertEquals(pms, psl.getPluginManagementService());
    }

    @Test public void getPubSubService() throws Exception {
        assertEquals(as, psl.getArbitratorService());
    }

    private PluginServiceLocator psl;
    private ArbitratorService as;
    private PluginManagementService pms;
    private IPubSubService pss;
}