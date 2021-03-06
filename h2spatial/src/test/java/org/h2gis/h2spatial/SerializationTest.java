/**
 * h2spatial is a library that brings spatial support to the H2 Java database.
 *
 * h2spatial is distributed under GPL 3 license. It is produced by the "Atelier SIG"
 * team of the IRSTV Institute <http://www.irstv.fr/> CNRS FR 2488.
 *
 * Copyright (C) 2007-2012 IRSTV (FR CNRS 2488)
 *
 * h2patial is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * h2spatial is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * h2spatial. If not, see <http://www.gnu.org/licenses/>.
 *
 * For more information, please consult: <http://www.orbisgis.org/>
 * or contact directly:
 * info_at_ orbisgis.org
 */

package org.h2gis.h2spatial;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.io.WKBReader;
import org.h2gis.h2spatial.internal.function.spatial.convert.ST_GeomFromText;
import org.h2gis.h2spatial.ut.SpatialH2UT;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;

import java.net.URL;
import java.sql.Connection;
import java.sql.ResultSet;
import java.sql.Statement;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Geometry fields serialization tests.
 * @author Nicolas Fortin
 */
public class SerializationTest {
    private static Connection connection;
    private static final String DB_NAME = "SerializationTest";

    @BeforeClass
    public static void tearUp() throws Exception {
        // Keep a connection alive to not close the DataBase on each unit test
        connection = SpatialH2UT.createSpatialDataBase(DB_NAME);
        // Set up test data
        URL sqlURL = OGCConformance1Test.class.getResource("ogc_conformance_test3.sql");
        Statement st = connection.createStatement();
        st.execute("drop table if exists spatial_ref_sys;");
        st.execute("RUNSCRIPT FROM '"+sqlURL+"'");
        // Close the DataBase then reopen it
        connection.close();
        connection = SpatialH2UT.openSpatialDataBase(DB_NAME);
    }

    @AfterClass
    public static void tearDown() throws Exception {
        connection.close();
    }

    /**
     *  For this test, we will check to see that all of the feature tables are
     *  represented by entries in the GEOMETRY_COLUMNS table/view.
     *  @throws Exception
     */
    @Test
    public void T1() throws Exception {
        Statement st = connection.createStatement();
        ResultSet rs = st.executeQuery("SELECT f_table_name FROM geometry_columns;");
        Set<String> tablesWithGeometry = new HashSet<String>(11);
        while(rs.next()) {
            tablesWithGeometry.add(rs.getString("f_table_name").toLowerCase());
        }
        assertTrue(tablesWithGeometry.contains("lakes"));
        assertTrue(tablesWithGeometry.contains("road_segments"));
        assertTrue(tablesWithGeometry.contains("divided_routes"));
        assertTrue(tablesWithGeometry.contains("buildings"));
        assertTrue(tablesWithGeometry.contains("forests"));
        assertTrue(tablesWithGeometry.contains("bridges"));
        assertTrue(tablesWithGeometry.contains("named_places"));
        assertTrue(tablesWithGeometry.contains("streams"));
        assertTrue(tablesWithGeometry.contains("ponds"));
        assertTrue(tablesWithGeometry.contains("map_neatlines"));
    }

    /**
     * For this test, we will determine the SRID of Goose Island.
     * @throws Exception
     */
    @Test
    public void sridTest() throws Exception {
        Statement st = connection.createStatement();
        ResultSet rs = st.executeQuery("SELECT ST_SRID(boundary) FROM named_places WHERE name = 'Goose Island'");
        assertTrue(rs.next());
        assertEquals(101, rs.getInt(1));
    }

    /**
     * For this test, varchar cast geometry in Route 75.
     * @throws Exception
     */
    @Test
    public void CastTest() throws Exception {
        Statement st = connection.createStatement();
        ResultSet rs = st.executeQuery("SELECT boundary::TEXT FROM named_places WHERE name = 'Goose Island';");
        assertTrue(rs.next());
        assertEquals("POLYGON ((67 13, 67 18, 59 18, 59 13, 67 13))", rs.getString(1));
    }

    /**
     * For this test, varchar cast of the union of Blue Lake and Goose Island.
     * @throws Exception
     */
    @Test
    public void CastTest2() throws Exception {
        Statement st = connection.createStatement();
        ResultSet rs = st.executeQuery("SELECT ST_Union(shore, boundary) FROM lakes, named_places " +
                "WHERE lakes.name = 'Blue Lake' AND named_places.name = 'Goose Island'");
        assertTrue(rs.next());
        assertEquals("POLYGON ((52 18, 66 23, 73 9, 48 6, 52 18))", rs.getString(1));
    }
}
