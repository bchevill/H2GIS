/*
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

import com.vividsolutions.jts.io.WKTReader;
import org.h2gis.h2spatial.ut.SpatialH2UT;
import org.junit.AfterClass;
import org.junit.BeforeClass;
import org.junit.Test;
import org.h2gis.utilities.SFSUtilities;
import org.h2gis.utilities.SpatialResultSet;
import java.sql.Connection;
import java.sql.ResultSet;
import java.sql.SQLException;
import java.sql.Statement;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Test spatial wrapper of Connection
 * @author Nicolas Fortin
 */
public class ConnectionWrapTest {
    private static Connection connection;
    private static final String DB_NAME = "ConnectionWrapTest";

    @BeforeClass
    public static void tearUp() throws Exception {
        // Keep a connection alive to not close the DataBase on each unit test
        connection = SFSUtilities.wrapConnection(SpatialH2UT.createSpatialDataBase(DB_NAME));
    }
    @AfterClass
    public static void tearDown() throws Exception {
        connection.close();
    }

    @Test
    public void testGeometryCast() throws SQLException {
        Statement stat = connection.createStatement();
        stat.execute("DROP TABLE IF EXISTS AREA");
        stat.execute("create table area(idarea int primary key, the_geom POLYGON)");
        stat.execute("insert into area values(1, 'POLYGON ((-10 109, 90 109, 90 9, -10 9, -10 109))')");
        stat.execute("insert into area values(2, 'POLYGON ((90 109, 190 109, 190 9, 90 9, 90 109))')");
        SpatialResultSet rs = stat.executeQuery("select idarea, the_geom  from area").unwrap(SpatialResultSet.class);
        assertTrue(rs.next());
        assertEquals("POLYGON ((-10 109, 90 109, 90 9, -10 9, -10 109))", rs.getGeometry("the_geom").toText());
        assertEquals("POLYGON ((-10 109, 90 109, 90 9, -10 9, -10 109))", rs.getGeometry(2).toText());
        assertEquals("POLYGON ((-10 109, 90 109, 90 9, -10 9, -10 109))", rs.getGeometry().toText());
        rs.close();
        stat.execute("DROP TABLE AREA");
    }

    @Test
    public void testGeometryUpdate() throws Exception {
        WKTReader wktReader = new WKTReader();
        Statement stat = connection.createStatement(ResultSet.TYPE_FORWARD_ONLY ,ResultSet.CONCUR_UPDATABLE);
        stat.execute("DROP TABLE IF EXISTS AREA");
        stat.execute("create table area(idarea int primary key, the_geom POLYGON)");
        SpatialResultSet rs = stat.executeQuery("select * from area").unwrap(SpatialResultSet.class);
        rs.moveToInsertRow();
        rs.updateInt(1, 1);
        rs.updateGeometry(2, wktReader.read("POLYGON ((-10 109, 90 109, 90 9, -10 9, -10 109))"));
        rs.insertRow();
        rs.moveToInsertRow();
        rs.updateInt(1, 2);
        rs.updateGeometry("the_geom", wktReader.read("POLYGON ((90 109, 190 109, 190 9, 90 9, 90 109))"));
        rs.insertRow();
        rs.close();
        rs = connection.createStatement().executeQuery("select * from area").unwrap(SpatialResultSet.class);
        assertTrue(rs.next());
        assertEquals("POLYGON ((-10 109, 90 109, 90 9, -10 9, -10 109))", rs.getGeometry("the_geom").toText());
        assertTrue(rs.next());
        assertEquals("POLYGON ((90 109, 190 109, 190 9, 90 9, 90 109))", rs.getGeometry("the_geom").toText());
        assertFalse(rs.next());
        stat.execute("DROP TABLE AREA");
    }
}
