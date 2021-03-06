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

package org.h2gis.h2spatialext.function.spatial.create;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.geom.Polygon;
import com.vividsolutions.jts.util.GeometricShapeFactory;
import org.h2gis.h2spatialapi.DeterministicScalarFunction;

import java.sql.SQLException;


/**
 * ST_MakeEllipse constructs an elliptical POLYGON with the given width and
 * height centered at the given point. Each ellipse contains 100 line segments.
 *
 * @author Adam Gouge
 * @author Erwan Bocher
 */
public class ST_MakeEllipse extends DeterministicScalarFunction {

    private static final GeometricShapeFactory GSF = new GeometricShapeFactory();

    public ST_MakeEllipse() {
        addProperty(PROP_REMARKS, "Constructs an elliptical POLYGON with the " +
                "given width and height centered at the given point. Each " +
                "ellipse contains 100 line segments.");
    }

    @Override
    public String getJavaStaticMethod() {
        return "makeEllipse";
    }

    /**
     * Make an ellipse centered at the given point with the given width and
     * height.
     *
     * @param p      Point
     * @param width  Width
     * @param height Height
     * @return An ellipse centered at the given point with the given width and height
     * @throws SQLException if the width or height is non-positive
     */
    public static Polygon makeEllipse(Point p, double width, double height) throws SQLException {
        if (height < 0 || width < 0) {
            throw new SQLException("Both width and height must be positive.");
        } else {
            GSF.setCentre(new Coordinate(p.getX(), p.getY()));
            GSF.setWidth(width);
            GSF.setHeight(height);
            return GSF.createEllipse();
        }
    }
}
